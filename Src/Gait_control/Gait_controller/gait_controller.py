import sys
import time
import threading
from pathlib import Path
from typing import Dict, List, Optional

FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from Src.Gait_control.Robot.robot_geometry_model import Spider_robot
from Src.Gait_control.Tripod_gait.tripod_gait import TripodGait


class GaitController:
    """Runs a tripod gait and publishes servo outputs."""

    def __init__(
        self,
        gait: Optional[TripodGait] = None,
        robot: Optional[Spider_robot] = None,
        control_hz: float = 50.0,
        pub_bind: Optional[str] = "tcp://*:5556",
        publisher_warmup: float = 0.2,
    ) -> None:
        self.robot = robot if robot is not None else Spider_robot()
        self.gait = gait if gait is not None else TripodGait()
        self.control_hz = float(control_hz) if control_hz > 0 else 50.0
        self.control_dt = 1.0 / self.control_hz
        self.pub_bind = pub_bind
        self.publisher_warmup = max(0.0, float(publisher_warmup))

        self._stop_event = threading.Event()
        self._loop_thread: Optional[threading.Thread] = None
        self._sequence = 0
        self._ctx = None
        self._pub = None
        self._zmq = None

    def start(self) -> None:
        if self._loop_thread and self._loop_thread.is_alive():
            return
        self._stop_event.clear()
        self._ensure_publisher()
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._loop_thread and self._loop_thread.is_alive():
            self._loop_thread.join()
        self._loop_thread = None
        self._close_publisher()

    def step(self, time_s: float, publish: bool = True) -> Dict[str, float]:
        positions = self.gait.sample(time_s)
        self._apply_positions(positions)
        servo_outputs = self.robot.read_servo_outputs() or {}
        if publish:
            self._publish(time_s, servo_outputs)
        return servo_outputs

    def close(self) -> None:
        self.stop()

    def _ensure_publisher(self) -> None:
        if not self.pub_bind or self._pub is not None:
            return
        try:
            import zmq
        except ImportError:
            print("[GaitController] pyzmq not installed, disabling broadcast")
            self.pub_bind = None
            return
        self._zmq = zmq
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(self.pub_bind)
        if self.publisher_warmup > 0:
            time.sleep(self.publisher_warmup)

    def _close_publisher(self) -> None:
        if self._pub is not None:
            try:
                self._pub.close(0)
            except Exception:
                pass
            self._pub = None
        if self._ctx is not None:
            try:
                self._ctx.term()
            except Exception:
                pass
            self._ctx = None
        self._zmq = None

    def _run_loop(self) -> None:
        base_time = time.perf_counter()
        next_tick = base_time
        while not self._stop_event.is_set():
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(max(0.0, next_tick - now))
                continue
            control_time = now - base_time
            try:
                self.step(control_time, publish=True)
            except Exception as exc:
                print(f"[GaitController] control loop error: {exc}")
            next_tick += self.control_dt
            if next_tick < now:
                next_tick = now + self.control_dt

    def _apply_positions(self, positions: Dict[str, List[float]]) -> None:
        for leg_name, target in positions.items():
            leg = self.robot.legs.get(leg_name)
            if leg is None:
                continue
            try:
                leg.write_end_coordinate(list(target))
            except Exception as exc:
                print(f"[GaitController] failed to update {leg_name}: {exc}")

    def _publish(self, time_s: float, servo_outputs: Dict[str, float]) -> None:
        if self._pub is None:
            return
        payload = {"seq": self._sequence, "t": time_s, "angles": servo_outputs}
        try:
            self._pub.send_json(payload)
            self._sequence += 1
        except Exception as exc:
            print(f"[GaitController] publish failed: {exc}")

