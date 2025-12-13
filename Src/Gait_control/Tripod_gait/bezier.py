import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# ——— 参数设置 ———
# 起点和终点示例 (可根据实际情况修改)

Y_STEP = 15
X_STEP = 20.0

initial_pos = [110, 0, -89.84]

p1 = [ initial_pos[0], initial_pos[1] - 2*Y_STEP, initial_pos[2] ]
p3 = [ initial_pos[0], initial_pos[1] + 2*Y_STEP, initial_pos[2] ]

z_lift = 10
z_down = -2

def Bezier(p1, p3, theta, z_lift, z_down):
    p1 = np.array(p1, dtype=float)
    p3 = np.array(p3, dtype=float)
    p2 = (p1 + p3) * 0.5

    # XY阶段参数
    if theta <= np.pi:
        s_xy = theta / np.pi
    else:
        s_xy = (2*np.pi - theta) / np.pi

    # Z阶段参数
    if theta <= np.pi:
        s_z = theta / np.pi
        p2[2] += z_down
    else:
        s_z = (theta - np.pi) / np.pi
        p2[2] += z_lift

    def mix_axis(c0, c2, c3, s):
        return ((1 - s)**4 + 4*(1 - s)**3*s) * c0 + \
               6*(1 - s)**2 * s**2       * c2 + \
               (4*(1 - s)*s**3 + s**4)    * c3

    x = mix_axis(p1[0], p2[0], p3[0], s_xy)
    y = mix_axis(p1[1], p2[1], p3[1], s_xy)
    z = mix_axis(p1[2], p2[2], p3[2], s_z)
    return np.array([x, y, z])




