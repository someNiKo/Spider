"""Test to run `mode1` from `servo_control_test.py` alongside the AngleMonitor TUI.

This script will:
- create a `servo` instance and inject it into the imported test module so
  that `mode1` can call it
- start `mode1` in a daemon thread (so the test can stop cleanly)
- start `AngleMonitor` to read `servo.read_joint_angle()` at a modest rate
- run for a few seconds then stop the monitor and the servo

Run this file from the repository root with Python.
"""

import time
import threading
from pathlib import Path
import sys

FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from Tests import servo_control_test as sct
from Src.Visualization.angle_data_monitor import AngleMonitor


def run_test(duration_s: float = 10.0):
    # create servo instance and inject into servo_control_test module
    servo_inst = sct.servo_control.servo()
    sct.servo = servo_inst
    servo_inst.start()

    # start mode1 in a daemon thread
    mode1_thread = threading.Thread(
        target=lambda: sct.mode1(amp=30.0, freq=0.2, base=90.0, send_hz=400.0),
        daemon=True,
        name="mode1-thread",
    )


    try:
        mode1_thread.start()
        monitor = AngleMonitor(lambda: servo_inst.read_joint_angle(), refresh_hz=10.0)
    except KeyboardInterrupt:
        print("Stopping monitor and servo...")
        try:
            monitor.stop()
            monitor.join(timeout=1.0)
        except Exception:
            pass
        try:
            servo_inst.reset_joint_angle()
        except Exception:
            pass
        try:
            servo_inst.stop()
        except Exception:
            pass        


if __name__ == "__main__":
    run_test()
