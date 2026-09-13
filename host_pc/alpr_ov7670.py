#!/usr/bin/env python3
"""Receive the FPGA's OV7670 UDP video and run FastALPR on the PC.

Complete frames are displayed with OpenCV and periodically submitted for plate
detection. New plate text is returned to the Nexys4 DDR OLED over USB-UART.
Camera capture, grayscale conversion, optional Sobel filtering, and the N4VD
UDP transport are implemented in FPGA RTL.
"""

import argparse
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np

# Must stay byte-for-byte compatible with eth_video_udp.v. The leading `>`
# means network byte order; all H fields are unsigned 16-bit integers.
MAGIC = b"N4VD"
# Format 2 is the legacy grayscale-only stream. Format 3 appends an inclusive
# Sobel-derived ROI and its edge score to every packet in the frame.
BASE_HEADER_FMT = ">4sHHHHHH"
BASE_HEADER_LEN = struct.calcsize(BASE_HEADER_FMT)
ROI_HEADER_FMT = ">HHHHH"
ROI_HEADER_LEN = struct.calcsize(ROI_HEADER_FMT)
PIX_PER_PKT = 192
PLATE_CHARS = set("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789- ")


def gray_to_bgr(pixels):
    return cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)


def padded_roi(roi, width, height, padding):
    """Convert an inclusive FPGA ROI to a clipped, padded inclusive ROI."""
    if roi is None:
        return None
    x0, y0, x1, y1, score = roi
    if not (0 <= x0 <= x1 < width and 0 <= y0 <= y1 < height and score > 0):
        return None
    roi_w = x1 - x0 + 1
    roi_h = y1 - y0 + 1
    pad_x = max(8, int(round(roi_w * padding)))
    pad_y = max(8, int(round(roi_h * padding)))
    return (
        max(0, x0 - pad_x),
        max(0, y0 - pad_y),
        min(width - 1, x1 + pad_x),
        min(height - 1, y1 + pad_y),
        score,
    )


def sanitize_plate(text):
    out = []
    for ch in text.upper():
        if ch in PLATE_CHARS:
            out.append(ch)
        if len(out) >= 16:
            break
    return "".join(out).strip()


def plate_from_results(results):
    best_text = None
    best_score = -1.0
    for result in results:
        ocr = getattr(result, "ocr", None)
        if ocr is None:
            continue
        text = sanitize_plate(str(getattr(ocr, "text", "") or ""))
        if not text:
            continue
        det = getattr(result, "detection", None)
        score = float(getattr(det, "confidence", 0.0) or 0.0)
        if score > best_score:
            best_score = score
            best_text = text
    return best_text, best_score


def _port_blob(port):
    return " ".join(
        str(x) for x in (port.device, port.description, port.manufacturer, port.hwid)
    ).upper()


def _is_digilent_uart(port):
    blob = _port_blob(port)
    if not any(tag in blob for tag in ("FTDI", "FT2232", "DIGILENT")):
        return False
    # Nexys USB-UART is FT2232 channel B (interface 1), not JTAG channel A.
    name = port.device
    return name.endswith("1") or "MI_01" in blob or "IF=01" in blob


def find_serial_port(preferred):
    try:
        from serial.tools import list_ports
    except ImportError:
        return preferred
    if preferred:
        return preferred
    ports = list(list_ports.comports())
    for port in ports:
        if _is_digilent_uart(port):
            return port.device
    for port in ports:
        if any(tag in _port_blob(port) for tag in ("FTDI", "FT2232", "DIGILENT", "USB SERIAL")):
            return port.device
    return ports[0].device if ports else None


def open_serial(port, baud):
    if port is None:
        print("No serial port. OLED will not update. Use --serial COMx")
        return None
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is missing")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as exc:
        print("Could not open %s (%s). OLED will not update." % (port, exc))
        return None
    return ser


def send_plate(ser, plate):
    if ser is None:
        return
    try:
        ser.write((plate + "\n").encode("ascii"))
        ser.flush()
    except Exception as exc:
        print("serial write failed:", exc)


class AlprWorker(object):
    """Run relatively slow detector/OCR inference away from UDP reception.

    Only the newest submitted frame is retained, preventing an ever-growing
    queue when inference takes longer than the camera frame interval.
    """

    def __init__(self, use_roi=True, roi_padding=0.15,
                 detector_model="yolo-v9-t-384-license-plate-end2end",
                 ocr_model="cct-s-v2-global-model"):
        self.lock = threading.Lock()
        self.frame = None
        self.error = None
        self.plate = None
        self.score = 0.0
        self.roi_used = False
        self.use_roi = use_roi
        self.roi_padding = roi_padding
        self.detector_model = detector_model
        self.ocr_model = ocr_model
        self._stop = False

    def submit(self, img, roi=None):
        copy = img.copy()
        with self.lock:
            self.frame = (copy, roi)

    def snapshot(self):
        with self.lock:
            return self.error, self.plate, self.score, self.roi_used

    def stop(self):
        self._stop = True

    def run(self):
        try:
            from fast_alpr import ALPR
        except ImportError:
            self.error = "fast-alpr is missing"
            return
        try:
            alpr = ALPR(
                detector_model=self.detector_model,
                ocr_model=self.ocr_model,
            )
        except Exception as exc:
            self.error = "FastALPR init failed: %s" % exc
            return
        while not self._stop:
            with self.lock:
                work = self.frame
                self.frame = None
            if work is None:
                time.sleep(0.05)
                continue
            frame, roi = work
            try:
                roi_box = padded_roi(
                    roi, frame.shape[1], frame.shape[0], self.roi_padding
                ) if self.use_roi else None
                inference_frame = frame
                used_roi = False
                if roi_box is not None:
                    x0, y0, x1, y1, _ = roi_box
                    inference_frame = frame[y0:y1 + 1, x0:x1 + 1]
                    used_roi = True
                if hasattr(alpr, "predict"):
                    results = alpr.predict(inference_frame)
                else:
                    results = alpr.draw_predictions(inference_frame).results
                plate, score = plate_from_results(results)
                # A false Sobel proposal must not make recognition worse.
                if plate is None and used_roi:
                    if hasattr(alpr, "predict"):
                        results = alpr.predict(frame)
                    else:
                        results = alpr.draw_predictions(frame).results
                    plate, score = plate_from_results(results)
                    used_roi = False
            except Exception as exc:
                print("ALPR error:", exc)
                continue
            with self.lock:
                self.plate = plate
                self.score = score
                self.roi_used = used_roi


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--serial", default="", help="USB-UART COM port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Seconds between ALPR attempts")
    parser.add_argument("--no-roi", action="store_true",
                        help="Ignore format-3 FPGA ROI metadata")
    parser.add_argument("--roi-padding", type=float, default=0.15,
                        help="Fractional padding around the FPGA ROI (default: 0.15)")
    parser.add_argument("--ocr-model", default="cct-s-v2-global-model",
                        help="FastALPR OCR model (default: cct-s-v2-global-model)")
    parser.add_argument("--detector-model",
                        default="yolo-v9-t-384-license-plate-end2end",
                        help="FastALPR detector model")
    args = parser.parse_args()

    if sys.version_info < (3, 10):
        sys.exit("FastALPR needs Python 3.10 or newer")

    ser = open_serial(find_serial_port(args.serial or None), args.baud)
    if args.roi_padding < 0:
        parser.error("--roi-padding must be non-negative")

    worker = AlprWorker(
        use_roi=not args.no_roi,
        roi_padding=args.roi_padding,
        detector_model=args.detector_model,
        ocr_model=args.ocr_model,
    )
    threading.Thread(target=worker.run, daemon=True).start()

    # A large receive buffer helps absorb a burst containing 1,600 datagrams.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    try:
        sock.bind((args.bind, args.port))
    except OSError as exc:
        sys.exit(
            "Could not bind %s:%d (%s).\n"
            "Another program may already be using UDP port %d."
            % (args.bind, args.port, exc, args.port)
        )
    sock.settimeout(0.5)

    cv2.namedWindow("OV7670 ALPR", cv2.WINDOW_NORMAL)

    width = 640
    height = 480
    pkt_cnt = (width * height) // PIX_PER_PKT
    frame = np.zeros(width * height, dtype=np.uint8)
    have_pixels = False
    cur_id = None
    cur_roi = None
    display_roi = None
    last_show = time.time()
    last_alpr = 0.0
    last_sent = ""
    display = np.zeros((480, 640, 3), dtype=np.uint8)

    try:
        while True:
            # Reception stays in the main thread so slow inference cannot fill
            # the operating-system UDP queue and make the displayed video lag.
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break
                continue

            if len(data) < BASE_HEADER_LEN + 1:
                continue
            # Decode and validate the header generated by eth_video_udp.v before
            # copying this payload into its linear location in the frame.
            magic, frame_id, pkt_idx, hdr_cnt, w, h, fmt = struct.unpack(
                BASE_HEADER_FMT, data[:BASE_HEADER_LEN]
            )
            if magic != MAGIC or fmt not in (2, 3) or w == 0 or h == 0:
                continue
            header_len = BASE_HEADER_LEN
            packet_roi = None
            if fmt == 3:
                header_len += ROI_HEADER_LEN
                if len(data) < header_len + 1:
                    continue
                x0, y0, x1, y1, roi_score = struct.unpack(
                    ROI_HEADER_FMT, data[BASE_HEADER_LEN:header_len]
                )
                if roi_score:
                    packet_roi = (x0, y0, x1, y1, roi_score)
            if w != width or h != height or hdr_cnt != pkt_cnt:
                width, height, pkt_cnt = int(w), int(h), int(hdr_cnt)
                frame = np.zeros(width * height, dtype=np.uint8)
                have_pixels = False
                cur_id = None

            if pkt_idx >= pkt_cnt:
                continue

            do_show = False
            if cur_id is None:
                cur_id = frame_id
                cur_roi = packet_roi
            elif frame_id != cur_id:
                # A new frame ID closes the previous frame. Display partial
                # frames when packets were lost rather than freezing the UI.
                if have_pixels:
                    display = gray_to_bgr(frame.reshape(height, width))
                    display_roi = cur_roi
                    last_show = time.time()
                    do_show = True
                    if time.time() - last_alpr >= args.interval:
                        worker.submit(display, display_roi)
                        last_alpr = time.time()
                frame.fill(0)
                have_pixels = False
                cur_id = frame_id
                cur_roi = packet_roi
            elif packet_roi is not None:
                cur_roi = packet_roi

            pixels = np.frombuffer(data[header_len:], dtype=np.uint8)
            start = int(pkt_idx) * PIX_PER_PKT
            # Packet index maps directly to a 192-pixel slice of the linear
            # framebuffer used by the FPGA packetizer.
            n = min(len(pixels), PIX_PER_PKT, frame.size - start)
            if n > 0:
                frame[start:start + n] = pixels[:n]
                have_pixels = True

            now = time.time()
            if now - last_show > 0.5 and have_pixels and cur_id is not None:
                display = gray_to_bgr(frame.reshape(height, width))
                display_roi = cur_roi
                last_show = now
                do_show = True
                if now - last_alpr >= args.interval:
                    worker.submit(display, display_roi)
                    last_alpr = now

            if not do_show:
                continue

            err, plate, score, roi_used = worker.snapshot()
            if err:
                print(err)
                break

            if plate and plate != last_sent:
                send_plate(ser, plate)
                last_sent = plate

            # Keep an unannotated image for manual inference with the spacebar.
            clean_display = display.copy()
            shown_roi = padded_roi(display_roi, width, height, args.roi_padding)
            if shown_roi is not None and not args.no_roi:
                x0, y0, x1, y1, roi_edge_score = shown_roi
                cv2.rectangle(display, (x0, y0), (x1, y1), (0, 255, 255), 2)
                cv2.putText(
                    display, "Sobel ROI %d" % roi_edge_score, (x0, max(42, y0 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA
                )
            label = "%s  %.0f%%%s" % (
                plate, score * 100, " ROI" if roi_used else ""
            ) if plate else "no plate"
            cv2.putText(
                display, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0), 2, cv2.LINE_AA
            )
            cv2.imshow("OV7670 ALPR", display)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key == ord(" "):
                worker.submit(clean_display, display_roi)
                last_alpr = time.time()
    finally:
        worker.stop()
        sock.close()
        if ser is not None:
            ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
