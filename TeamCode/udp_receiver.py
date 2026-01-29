#!/usr/bin/env python3
"""
Nhận UDP log (type 0x00) và video JPEG (type 0x01) từ robot FTC.
Hai cửa sổ: UDP Video + Log; log in ra console.
Tự động ghi nhận sender (IP:port) từ gói nhận được.
Đọc config từ udp_config.txt (cùng thư mục).
"""

import socket
import threading
import os
import sys
import logging

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


def setup_logging(log_to_file=False):
    """Cấu hình logging: console + tùy chọn file."""
    log_format = "%(asctime)s [%(levelname)s] %(message)s"
    date_fmt = "%H:%M:%S"
    handlers = [logging.StreamHandler(sys.stdout)]
    if log_to_file:
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "udp_receiver.log")
        handlers.append(logging.FileHandler(log_path, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format=log_format, datefmt=date_fmt, handlers=handlers)


def load_config():
    """Parse udp_config.txt. Return dict: udp_port, bind_address, log_to_file."""
    config = {"udp_port": 5000, "bind_address": "0.0.0.0", "log_to_file": False}
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
                elif key == "log_to_file":
                    config["log_to_file"] = value.lower() in ("1", "true", "yes")
    return config


def run_receiver(port, bind_address):
    log_lines = []
    log_lock = threading.Lock()
    current_frame = None
    frame_lock = threading.Lock()
    senders = set()  # (ip, port) đã gửi tới
    sender_lock = threading.Lock()
    last_sender_str = [""]  # [0] = "IP:port" gửi gần nhất (để hiển thị)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_address, port))
    sock.settimeout(0.5)
    logging.info("Socket bound %s:%s", bind_address, port)

    def recv_loop():
        nonlocal current_frame
        while True:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as e:
                logging.warning("Socket error: %s", e)
                break
            if len(data) < 1:
                continue
            ip, port_s = addr[0], addr[1]
            with sender_lock:
                key = (ip, port_s)
                if key not in senders:
                    senders.add(key)
                    logging.info("Sender detected: %s:%s", ip, port_s)
                last_sender_str[0] = f"{ip}:{port_s}"
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
                logging.debug("Log from %s: %s", last_sender_str[0], text[:80])
                print(text)
            elif msg_type == TYPE_JPEG:
                try:
                    arr = np.frombuffer(payload, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        with frame_lock:
                            current_frame = img
                except Exception as e:
                    logging.debug("JPEG decode error: %s", e)

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
            with sender_lock:
                sender_label = last_sender_str[0] or "No sender yet"
                senders_count = len(senders)
            log_img = np.ones((LOG_WINDOW_HEIGHT, LOG_WINDOW_WIDTH, 3), dtype=np.uint8)
            log_img[:] = (250, 250, 250)
            y = 24
            cv2.putText(log_img, f"From: {sender_label}", (8, y), FONT, FONT_SCALE, (0, 100, 0), FONT_THICKNESS)
            y += 20
            if senders_count > 1:
                cv2.putText(log_img, f"Senders: {senders_count}", (8, y), FONT, FONT_SCALE * 0.9, (80, 80, 80), FONT_THICKNESS)
                y += 18
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
        logging.info("Receiver stopped.")


def main():
    config = load_config()
    setup_logging(config.get("log_to_file", False))
    port = config["udp_port"]
    bind_address = config["bind_address"]
    logging.info("UDP receiver: bind %s:%s", bind_address, port)
    logging.info("Two windows: UDP Video, UDP Log. ESC to exit. Sender range auto-detected.")
    run_receiver(port, bind_address)


if __name__ == "__main__":
    main()
    sys.exit(0)
