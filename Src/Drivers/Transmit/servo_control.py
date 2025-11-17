# -*- coding: utf-8 -*-
"""
Host controller for 18 servos over UART.
Protocol:
  [0]  0xAA
  [1]  0x55
  [2]  LEN = 41  (from CMD .. CRC)
  [3]  CMD = 0x01 (set 18 angles)
  [4..5]  SEQ (le)
  [6..41] angles[18] (each uint16, unit=0.1°; e.g. 900 == 90.0°)
  [42..43] CRC16-IBM (le, poly=0xA001, init=0xFFFF) over bytes [3..41] (CMD..angles)
"""

import time
import math
import struct
import serial
from . import config as cfg
import threading
import queue


class servo:

    CMD_SET18 = 0x01
    LEN_FIXED = 41         # CMD..CRC length
    START1, START2 = 0xAA, 0x55

    def __init__(self, 
                 port: str = cfg.PORT, 
                 baud: int = cfg.BAUD,
                 control_frequency: int = cfg.CONTROL_FREQUENCE,
                 send_order: list = cfg.SEND_ORDER,
                 default_joint_angle: dict = cfg.DEFAULT_JOINT_ANGLE):
        self.port = port
        self.baud = baud
        self.control_frequency = control_frequency
        self.send_order = send_order
        self.DEFAULT_JOINT_ANGLE = default_joint_angle
        self.__joint_angle = default_joint_angle
        self._lock = threading.Lock()
        self._q = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = None
        self._ser = None
        self._seq = 0

    def set_angle(self, joint_name: str, joint_angle: float):
        try:
            ang = float(joint_angle)
        except (ValueError, TypeError) as e:
            print(f"set_angle: invalid angle for {joint_name}: {joint_angle} -> {e}")
            return
        ang = max(0.0, min(180.0, ang))
        with self._lock:
            self.__joint_angle[joint_name] = ang


    def set_all_angle(self, joint_angle: dict):
        if not isinstance(joint_angle, dict):
            return

        with self._lock:
            for k, v in joint_angle.items():
                try:
                    ang = float(v)
                except (ValueError, TypeError) as e:
                    print(f"set_all_angle: invalid angle for {k}: {v} -> {e}")
                    continue
                ang = max(0.0, min(180.0, ang))
                self.__joint_angle[k] = ang
            snapshot = dict(self.__joint_angle)

        # 非阻塞地把最新快照放入单槽队列（覆盖旧帧）
        try:
            self._q.put_nowait(snapshot)
        except queue.Full:
            try:
                _ = self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait(snapshot)
            except Exception as e:
                print("Warning: failed to update single-slot queue:", e)

    def read_joint_angle(self) -> dict:
        with self._lock:
            # print(self.__joint_angle)
            return dict(self.__joint_angle)
        
    def reset_joint_angle(self):
        with self._lock:
            self.set_all_angle(self.DEFAULT_JOINT_ANGLE)
        
    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=False)
        self._thread.start()


    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._ser:
            try:
                self._ser.close()
            except Exception as e:
                print("Warning: failed to close serial port:", e)   

    def _run(self):
        self._open_serial()
        period = 1.0 / float(self.control_frequency) if self.control_frequency and self.control_frequency > 0 else 1.0/150.0
        next_time = time.time()
        
        while not self._stop.is_set():
            angles_snapshot = None
            try:
                angles_snapshot = self._q.get_nowait()
            except queue.Empty:
                angles_snapshot = None

            if angles_snapshot is None:
                angles_snapshot = self.read_joint_angle()

            if self.send_order:
                angles_list = [angles_snapshot.get(name, 90.0) for name in self.send_order]
            else:
                angles_list = [angles_snapshot[k] for k in angles_snapshot]

            frame = self._build_frame(self._seq, angles_list)
            self._seq = (self._seq + 1) & 0xFFFF

            if self._ser:
                try:
                    self._ser.write(frame)
                except (serial.SerialException, OSError) as e:
                    print("serial write error:", e)
                    try:
                        self._ser.close()
                    except Exception as e2:
                        print("Warning: failed to close serial after write error:", e2)
                    self._ser = None

            if self._ser is None:
                self._open_serial()

            next_time += period
            sleep_t = next_time - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_time = time.time()

    def _open_serial(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0)
            print(f"Opened {self.port} @ {self.baud} baud. Sending {self.control_frequency:.0f} Hz servo frames…")
        except (serial.SerialException, OSError) as e:
            self._ser = None
            print("serial open failed:", e)   
    
    def _crc16_ibm(self, data: bytes) -> int:
        """CRC16-IBM (Modbus) poly=0xA001, init=0xFFFF"""
        crc = 0xFFFF
        for ch in data:
            crc ^= ch
            for _ in range(8):
                if (crc & 0x0001) != 0:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def _deg_to_u16_d10(self, deg: float) -> int:
        """angle(°) -> uint16(0.1°), limit to 0..180°"""
        if deg < 0.0: deg = 0.0
        if deg > 180.0: deg = 180.0
        return int(round(deg * 10.0)) & 0xFFFF

    def _build_frame(self, _seq: int, angles_deg_18):
        """
        seq: increase 1 when it is called. (check for frame drops)
        angles_deg_18: angles of all servos (°, float)
        return the entile frame in bytes
        """
        #  CMD, SEQ, angles
        payload = bytearray()
        payload.append(self.CMD_SET18)                       # CMD
        payload += struct.pack("<H", _seq & 0xFFFF)      # SEQ
        for a in angles_deg_18:
            payload += struct.pack("<H", self._deg_to_u16_d10(a))

        # CRC of CMD..angles
        crc = self._crc16_ibm(payload)
        payload += struct.pack("<H", crc)

        # head + LEN + payload
        frame = bytearray([self.START1, self.START2, self.LEN_FIXED])  # AA 55 LEN
        frame += payload                                # CMD..CRC
        return bytes(frame)

