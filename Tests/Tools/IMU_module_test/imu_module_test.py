#!/usr/bin/env python3
# coding: utf-8

import math
import sys
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from YbImuSerialLib import YbImuSerial


def open_imu(port=None, debug=False):
	if port:
		return YbImuSerial(port, debug=debug)

	if sys.platform.startswith("win"):
		for com_index in range(1, 257):
			port_name = f"COM{com_index}"
			try:
				return YbImuSerial(port_name, debug=debug)
			except Exception:
				continue
		raise RuntimeError("No COM port opened")

	port_list = [
		"COM5",
	]
	for port_name in port_list:
		try:
			return YbImuSerial(port_name, debug=debug)
		except Exception:
			continue
	raise RuntimeError("No serial port opened")


def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
	roll = math.radians(roll_deg)
	pitch = math.radians(pitch_deg)
	yaw = math.radians(yaw_deg)

	cr, sr = math.cos(roll), math.sin(roll)
	cp, sp = math.cos(pitch), math.sin(pitch)
	cy, sy = math.cos(yaw), math.sin(yaw)

	rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
	ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
	rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

	return rz @ ry @ rx


def build_cuboid(size=(1.2, 0.6, 0.2)):
	lx, ly, lz = size
	x = lx / 2.0
	y = ly / 2.0
	z = lz / 2.0

	vertices = np.array(
		[
			[-x, -y, -z],
			[x, -y, -z],
			[x, y, -z],
			[-x, y, -z],
			[-x, -y, z],
			[x, -y, z],
			[x, y, z],
			[-x, y, z],
		]
	)

	faces = [
		[0, 1, 2, 3],
		[4, 5, 6, 7],
		[0, 1, 5, 4],
		[2, 3, 7, 6],
		[1, 2, 6, 5],
		[0, 3, 7, 4],
	]
	return vertices, faces


def main():
	port = sys.argv[1] if len(sys.argv) > 1 else None
	imu = open_imu(port=port, debug=False)
	imu.create_receive_threading()

	time.sleep(0.1)
	try:
		imu.calibration_imu()
		imu.set_report_rate(100)
		imu.set_algo_type(6)
	except Exception:
		pass

	vertices, faces = build_cuboid()

	fig = plt.figure(figsize=(7, 6))
	ax = fig.add_subplot(111, projection="3d")
	ax.set_box_aspect([1, 1, 1])
	ax.set_xlim(-1.2, 1.2)
	ax.set_ylim(-1.2, 1.2)
	ax.set_zlim(-1.2, 1.2)
	ax.set_xlabel("X")
	ax.set_ylabel("Y")
	ax.set_zlabel("Z")
	ax.set_title("IMU Attitude")

	poly = Poly3DCollection([], facecolors="#4C78A8", edgecolors="#1D3557", alpha=0.8)
	ax.add_collection3d(poly)

	def update(_frame):
		roll, pitch, yaw = imu.get_imu_attitude_data(ToAngle=True)
		r = rotation_matrix(roll, pitch, yaw)
		rotated = (r @ vertices.T).T
		verts = [[rotated[idx] for idx in face] for face in faces]
		poly.set_verts(verts)
		ax.set_title(f"IMU Attitude  roll={roll:.1f}  pitch={pitch:.1f}  yaw={yaw:.1f}")
		return poly,

	anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
	plt.show()
	_ = anim


if __name__ == "__main__":
	main()
