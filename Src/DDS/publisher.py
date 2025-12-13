# ...existing code...
import threading
import time
import zmq
from typing import Callable, Optional

class Publisher:
    """Generic ZeroMQ PUB helper with explicit topics."""

    def __init__(
        self,
        bind: str = "tcp://*:6000",
        topic: str = "",
        publish_hz: float = 20.0,
        warmup: float = 0.2,
        add_meta: bool = True,
        auto_payload_cb: Optional[Callable[[], dict]] = None,
    ):
        self.bind = bind
        if not isinstance(topic, str) or not topic:
            raise ValueError("Publisher requires a non-empty topic string")
        self.default_topic = topic
        self.hz = publish_hz
        self.warmup = warmup
        self.add_meta = add_meta
        self._auto_cb = auto_payload_cb

        self._ctx = None
        self._sock = None
        self._thread = None
        self._stop = threading.Event()
        self._seq = 0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.bind(self.bind)
        if self.warmup > 0:
            time.sleep(self.warmup)
        self._thread = threading.Thread(target=self._loop, name="generic-pub", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        interval = 1.0 / self.hz if self.hz > 0 else 0.1
        while not self._stop.is_set():
            if self._auto_cb is not None:
                payload = self._auto_cb()
                if payload is not None:
                    # auto callback may return (payload) or (topic, payload)
                    if isinstance(payload, tuple) and len(payload) == 2:
                        topic_override, p = payload
                        self.publish_once(p, topic=topic_override)
                    else:
                        self.publish_once(payload)
            if self._stop.wait(interval):
                break

    def publish_once(self, payload: dict, topic: Optional[str] = None) -> None:
        if self._sock is None:
            raise RuntimeError("publisher not started")
        target_topic = topic or self.default_topic
        if not isinstance(target_topic, str) or not target_topic:
            raise ValueError("publish_once requires a non-empty topic")
        if self.add_meta:
            frame = {"seq": self._seq, "t": time.time(), **payload}
            self._seq += 1
        else:
            frame = payload
        try:
            self._sock.send_string(target_topic, zmq.SNDMORE)
            self._sock.send_json(frame)
        except Exception as e:
            # best-effort logging
            print(f"[GenericPublisher] send failed: {e}")

    def stop(self, timeout: float = 1.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=timeout)
        try:
            if self._sock:
                self._sock.close(0)
            if self._ctx:
                self._ctx.term()
        except Exception:
            pass