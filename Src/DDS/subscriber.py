# ...existing code...
import threading
import zmq
from typing import Callable, Optional

class Subscriber:
    """Generic ZeroMQ SUB helper enforcing explicit topics."""

    def __init__(
        self,
        connect: str = "tcp://127.0.0.1:6000",
        on_message: Optional[Callable[[dict], None]] = None,
        topic: str = "",
        recv_timeout: float = 2.0,
    ):
        self.connect = connect
        self.on_message = on_message
        if not isinstance(topic, str) or not topic:
            raise ValueError("Subscriber requires a non-empty topic string")
        self.topic = topic
        self.recv_timeout = recv_timeout

        self._ctx = None
        self._sock = None
        self._thread = None
        self._stop = threading.Event()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.connect(self.connect)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, self.topic)
        if self.recv_timeout and self.recv_timeout > 0:
            self._sock.setsockopt(zmq.RCVTIMEO, int(self.recv_timeout * 1000))
        self._thread = threading.Thread(target=self._loop, name="generic-sub", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                topic = self._sock.recv_string()
                payload = self._sock.recv_json()
            except zmq.Again:
                continue
            except Exception as e:
                print(f"[GenericSubscriber] recv error: {e}")
                continue
            try:
                if self.on_message:
                    try:
                        self.on_message(payload, topic)
                    except TypeError:
                        self.on_message(payload)
            except Exception as e:
                print(f"[GenericSubscriber] callback error: {e}")

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