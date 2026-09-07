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
# means network byte order; all six H fields are unsigned 16-bit integers.
MAGIC = b"N4VD"
HEADER_FMT = ">4sHHHHHH"
HEADER_LEN = struct.calcsize(HEADER_FMT)
PIX_PER_PKT = 192
PLATE_CHARS = set("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789- ")


def gray_to_bgr(pixels):
    return cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)


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

    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.error = None
        self.plate = None
        self.score = 0.0
        self._stop = False

    def submit(self, img):
        copy = img.copy()
        with self.lock:
            self.frame = copy

    def snapshot(self):
        with self.lock:
            return self.error, self.plate, self.score

    def stop(self):
        self._stop = True

    def run(self):
        try:
            from fast_alpr import ALPR
        except ImportError:
            self.error = "fast-alpr is missing. pip install -r pc/requirements-alpr.txt"
            return
        try:
            alpr = ALPR(
                detector_model="yolo-v9-t-384-license-plate-end2end",
                ocr_model="cct-xs-v2-global-model",
            )
        except Exception as exc:
            self.error = "FastALPR init failed: %s" % exc
            return
        while not self._stop:
            with self.lock:
                frame = self.frame
                self.frame = None
            if frame is None:
                time.sleep(0.05)
                continue
            try:
                if hasattr(alpr, "predict"):
                    results = alpr.predict(frame)
                else:
                    results = alpr.draw_predictions(frame).results
                plate, score = plate_from_results(results)
            except Exception as exc:
                print("ALPR error:", exc)
                continue
            with self.lock:
                self.plate = plate
                self.score = score


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--serial", default="", help="USB-UART COM port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Seconds between ALPR attempts")
    args = parser.parse_args()

    if sys.version_info < (3, 10):
        sys.exit("FastALPR needs Python 3.10 or newer")

    ser = open_serial(find_serial_port(args.serial or None), args.baud)
    worker = AlprWorker()
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

            if len(data) < HEADER_LEN + 1:
                continue
            # Decode and validate the header generated by eth_video_udp.v before
            # copying this payload into its linear location in the frame.
            magic, frame_id, pkt_idx, hdr_cnt, w, h, fmt = struct.unpack(
                HEADER_FMT, data[:HEADER_LEN]
            )
            if magic != MAGIC or fmt != 2 or w == 0 or h == 0:
                continue
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
            elif frame_id != cur_id:
                # A new frame ID closes the previous frame. Display partial
                # frames when packets were lost rather than freezing the UI.
                if have_pixels:
                    display = gray_to_bgr(frame.reshape(height, width))
                    last_show = time.time()
                    do_show = True
                    if time.time() - last_alpr >= args.interval:
                        worker.submit(display)
                        last_alpr = time.time()
                frame.fill(0)
                have_pixels = False
                cur_id = frame_id

            pixels = np.frombuffer(data[HEADER_LEN:], dtype=np.uint8)
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
                last_show = now
                do_show = True

            if not do_show:
                continue

            err, plate, score = worker.snapshot()
            if err:
                print(err)
                break

            if plate and plate != last_sent:
                send_plate(ser, plate)
                last_sent = plate

            label = "%s  %.0f%%" % (plate, score * 100) if plate else "no plate"
            cv2.putText(
                display, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0), 2, cv2.LINE_AA
            )
            cv2.imshow("OV7670 ALPR", display)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key == ord(" "):
                worker.submit(display)
                last_alpr = time.time()
    finally:
        worker.stop()
        sock.close()
        if ser is not None:
            ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
