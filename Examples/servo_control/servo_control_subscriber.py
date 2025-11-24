"""ZeroMQ subscriber that runs servo control and applies incoming angles.

This script starts a `servo_control.servo()` instance (and calls `start()`),
connects to a PUB socket and receives JSON messages with joint-angle dicts.
It applies received angles with `set_all_angle` if available.

Usage:
    python Tests/servo_control_subscriber.py --connect tcp://127.0.0.1:5556

Note: run this in a separate terminal/process from the publisher.
"""
from pathlib import Path
import sys
import time
import zmq
import argparse

FILE = Path(__file__).resolve()
ROOT = FILE.parents[2]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--connect", type=str, default="tcp://127.0.0.1:5556", help="PUB connect address")
    args = p.parse_args()

    from Src.Drivers.Transmit import servo_control

    servo = servo_control.servo()
    servo.start()

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(args.connect)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    # set a receive timeout so we can handle KeyboardInterrupt gracefully
    sub.setsockopt(zmq.RCVTIMEO, 1000)

    print(f"Subscriber connected to {args.connect}, servo started")

    try:
        while True:
            try:
                msg = sub.recv_json()
            except zmq.Again:
                # timeout, loop to allow KeyboardInterrupt
                continue

            if not isinstance(msg, dict):
                print("unexpected message format", msg)
                continue

            angles = None
            if 'angles' in msg and isinstance(msg['angles'], dict):
                angles = msg['angles']
            else:
                # maybe the publisher sent angles at top-level
                # accept any dict where values are numbers
                if all(isinstance(v, (int, float)) for v in msg.values()):
                    angles = msg

            if angles:
                try:
                    servo.set_all_angle(angles)
                except Exception as e:
                    print("failed to set angles:", e)

    except KeyboardInterrupt:
        print("subscriber stopping")
    finally:
        try:
            servo.reset_joint_angle()
        except Exception:
            pass
        try:
            servo.stop()
        except Exception:
            pass
        sub.close()
        ctx.term()

if __name__ == "__main__":
    main()
