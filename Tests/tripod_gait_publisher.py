"""Run the tripod gait controller and publish servo frames over ZeroMQ."""

import sys
import time
import signal
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from Src.Gait_control.Tripod_gait.tripod_gait import TripodGait
from Src.Gait_control.Gait_controller.gait_controller import GaitController


def main() -> None:
    gait = TripodGait()
    controller = GaitController(gait=gait, control_hz=200.0, pub_bind="tcp://*:6000")

    stop = False

    def _handle_sigint(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle_sigint)
    controller.start()
    print("Gait controller publishing on tcp://*:6000. Press Ctrl+C to stop.")

    try:
        while not stop:
            time.sleep(0.2)
    finally:
        controller.close()
        print("Controller stopped.")


if __name__ == "__main__":
    main()
