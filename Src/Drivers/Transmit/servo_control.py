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
import config as cfg
import threading







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
        self.__joint_angle = default_joint_angle
        self._lock = threading.Lock()

    def set_angle(joint_name: str, joint_angle: float):
        pass

    def set_all_angle(joint_angle: dict):
        pass

    def read_joint_angle(self) -> dict:
        # print(self.__joint_angle)
        return self.__joint_angle
        
    def start():
        pass

    def _run():
        while(True):
            pass

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

    def _build_frame(self, seq: int, angles_deg_18):
        """
        seq: increase 1 when it is called. (check for frame drops)
        angles_deg_18: angles of all servos (°, float)
        return the entile frame in bytes
        """
        #  CMD, SEQ, angles
        payload = bytearray()
        payload.append(self.CMD_SET18)                       # CMD
        payload += struct.pack("<H", seq & 0xFFFF)      # SEQ
        for a in angles_deg_18:
            payload += struct.pack("<H", self._deg_to_u16_d10(a))

        # CRC of CMD..angles
        crc = self._crc16_ibm(payload)
        payload += struct.pack("<H", crc)

        # head + LEN + payload
        frame = bytearray([self.START1, self.START2, self.LEN_FIXED])  # AA 55 LEN
        frame += payload                                # CMD..CRC
        return bytes(frame)






























# def test_all_servos():
#     '''
#     servos move in a sine wave. ±10°, 2Hz
#     '''
#     ser = serial.Serial(PORT, BAUD, timeout=0)
#     print(f"Opened {PORT} @ {BAUD} baud. Sending {SEND_HZ:.0f} Hz servo frames… Ctrl+C to stop.")
#     seq = 0
#     base = 90.0
#     amp = 10.0
#     freq = 2         
#     dt = 1.0 / SEND_HZ
#     t0 = time.time()

#     try:
#         while True:
#             t = time.time() - t0
#             val = base + amp * math.sin(2.0 * math.pi * freq * t)
#             angles = [val] * 18

#             frame = build_frame(seq, angles)
#             # 串口一次性写完
#             ser.write(frame)

#             seq = (seq + 1) & 0xFFFF
#             # 固定周期发送
#             time.sleep(dt)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         ser.close()
#         print("Serial closed.")

# ==== test ====
if __name__ == "__main__":
    s = servo()
    print(s.read_joint_angle())

