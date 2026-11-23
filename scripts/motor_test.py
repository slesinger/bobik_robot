#!/usr/bin/env python3
"""Spin the wheels briefly through the running bobik_driver (no ROS, no pyzmq needed).

Sends a ZMQ RADIO/DISH-over-UDP datagram (1 byte group length + group + payload)
with group "cmd_vel" and payload MsgCmdVel_t = 3x int16 LE (x, y, rotation) * 5000.
The Arduino has NO watchdog: it keeps the last command until it gets zero,
so the zero command is always sent at the end (also on Ctrl+C).

Usage: motor_test.py [x_m_s=0.1] [seconds=2] [host=192.168.1.21]
"""
import socket
import struct
import sys
import time

x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
host = sys.argv[3] if len(sys.argv) > 3 else "192.168.1.21"
PREC = 5000.0
GROUP = b"cmd_vel"

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send(vx, vy=0.0, rot=0.0):
    payload = struct.pack("<hhh", int(vx * PREC), int(-vy * PREC), int(rot * PREC))
    s.sendto(bytes([len(GROUP)]) + GROUP + payload, (host, 7654))


try:
    print(f"drive x={x} m/s for {secs} s -> {host}")
    end = time.time() + secs
    while time.time() < end:
        send(x)
        time.sleep(0.05)
finally:
    for _ in range(5):
        send(0.0)
        time.sleep(0.05)
    print("stopped (zero sent)")
