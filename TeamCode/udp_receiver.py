#!/usr/bin/env python3
"""
Nhận UDP log (type 0x00) và video JPEG (type 0x01) từ robot FTC.
Hai cửa sổ: UDP Video + Log; log in ra console.
Đọc config từ udp_config.txt (cùng thư mục).
"""

import socket
import threading
import time
import os
import sys

import cv2
import numpy as np

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "udp_config.txt")
TYPE_LOG = 0x00
TYPE_JPEG = 0x01
MAX_LOG_LINES = 30
LOG_WINDOW_WIDTH = 480
LOG_WINDOW_HEIGHT = 360
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.45
FONT_THICKNESS = 1


def load_config():
    """Parse udp_config.txt (key=value). Return dict with udp_port, bind_address."""
    config = {"udp_port": 5000, "bind_address": "0.0.0.0"}
    if not os.path.isfile(CONFIG_PATH):
        return config
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" in line:
                key, _, value = line.partition("=")
                key = key.strip().lower()
                value = value.strip()
                if key == "udp_port":
                    try:
                        config["udp_port"] = int(value)
                    except ValueError:
                        pass
                elif key == "bind_address":
                    config["bind_address"] = value
    return config


def run_receiver(port, bind_address):
    log_lines = []
    log_lock = threading.Lock()
    current_frame = None
    frame_lock = threading.Lock()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_address, port))
    sock.settimeout(0.5)

    def recv_loop():
        nonlocal current_frame
        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) < 1:
                continue
            msg_type = data[0]
            payload = data[1:]
            if msg_type == TYPE_LOG:
                try:
                    text = payload.decode("utf-8")
                except Exception:
                    text = payload.decode("utf-8", errors="replace")
                with log_lock:
                    for line in text.splitlines():
                        line = line.strip()
                        if line:
                            log_lines.append(line)
                            if len(log_lines) > MAX_LOG_LINES:
                                log_lines.pop(0)
                print(text)
            elif msg_type == TYPE_JPEG:
                try:
                    arr = np.frombuffer(payload, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        with frame_lock:
                            current_frame = img
                except Exception:
                    pass

    recv_thread = threading.Thread(target=recv_loop, daemon=True)
    recv_thread.start()

    cv2.namedWindow("UDP Video", cv2.WINDOW_NORMAL)
    cv2.namedWindow("UDP Log", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("UDP Log", LOG_WINDOW_WIDTH, LOG_WINDOW_HEIGHT)

    try:
        while True:
            with frame_lock:
                frame = current_frame
            if frame is not None:
                cv2.imshow("UDP Video", frame)
            else:
                blank = np.zeros((240, 320, 3), dtype=np.uint8)
                blank[:] = (40, 40, 40)
                cv2.putText(
                    blank, "Waiting for video...",
                    (20, 120), FONT, 0.6, (200, 200, 200), FONT_THICKNESS,
                )
                cv2.imshow("UDP Video", blank)

            with log_lock:
                lines = list(log_lines)
            log_img = np.ones((LOG_WINDOW_HEIGHT, LOG_WINDOW_WIDTH, 3), dtype=np.uint8)
            log_img[:] = (250, 250, 250)
            y = 24
            for line in lines[-MAX_LOG_LINES:]:
                if y + 20 > LOG_WINDOW_HEIGHT:
                    break
                cv2.putText(
                    log_img, line[:60] + ("..." if len(line) > 60 else ""),
                    (8, y), FONT, FONT_SCALE, (20, 20, 20), FONT_THICKNESS,
                )
                y += 18
            cv2.imshow("UDP Log", log_img)

            if cv2.waitKey(30) == 27:
                break
    finally:
        sock.close()
        cv2.destroyAllWindows()


def main():
    config = load_config()
    port = config["udp_port"]
    bind_address = config["bind_address"]
    print(f"UDP receiver: bind {bind_address}:{port}")
    print("Two windows: UDP Video, UDP Log. Console = log. ESC to exit.")
    run_receiver(port, bind_address)


if __name__ == "__main__":
    main()
    sys.exit(0)
