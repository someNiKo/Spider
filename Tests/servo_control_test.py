"""
mode 0: moving a single joint to specific angle with specific name
mode 1: testing all joint in sine wave
mode 2: testing a single joint with specific name in sine wave
"""

import sys
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[1] 
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from Src.Drivers.Transmit import servo_control
import time
import math

def mode0(joint_name: str, joint_angle: float, servo: servo_control.servo):
    try:
        while True:
            servo.set_angle(joint_name, joint_angle)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Control stoped")

def mode1(amp: float, freq: float, base: float, send_hz: float):
    snapshot = servo.read_joint_angle().copy()
    try:
        t = 0.0
        dt = 1.0 / send_hz
        while True:
            for joint_name in snapshot.keys():
                snapshot[joint_name] = base + amp * math.sin(2.0 * math.pi * freq * t)
            servo.set_all_angle(snapshot)
            t += dt
            time.sleep(dt)
    except KeyboardInterrupt:
        print("Control stoped")



def mode2(joint_name: str, amp: float, freq: float, base: float, send_hz: float):
    try:
        t = 0.0
        dt = 1.0 / send_hz
        while True:
            angle = base + amp * math.sin(2.0 * math.pi * freq * t)
            servo.set_angle(joint_name, angle)
            t += dt
            time.sleep(dt)
    except KeyboardInterrupt:
        print("Control stoped")

if __name__ == "__main__":
    import time

    servo = servo_control.servo()
    servo.start()

    try:
        #mode0("L1_femur", 120.0, servo)
        mode1(amp=50.0, freq=0.2, base=90.0, send_hz=400.0)
        #mode2(joint_name="L1_coxa", amp=50.0, freq=0.2, base=90.0, send_hz=400.0)
    except KeyboardInterrupt:
        servo.reset_joint_angle()

    servo.stop()