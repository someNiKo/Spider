# Paras.py

import numpy as np


INITIAL_PHASES = [0, np.pi]  # φ1(0)=0.0 rad, φ2(0)=1.0 rad

Z_LIFT   = 25    # 抬腿高度
Z_DOWN   = 3   # 支撑下压

Y_STEP = 20

X_STEP = 0

initial_pos = [0, 95, -120]
# ——— 每条腿的轨迹端点及所属振荡器 ———
LEG_CONFIG_Forward = {
    # osc = 0
    'leg2_left':  {
        'osc': 0,
        'P1' : [ initial_pos[0] + 2*Y_STEP, initial_pos[1], initial_pos[2] ],
        'P3' : [ initial_pos[0] - 2*Y_STEP, initial_pos[1], initial_pos[2] ]
    },
    'leg3_right': {
        'osc': 0,
        'P1' : [ initial_pos[0] - Y_STEP, initial_pos[1] - Y_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] + Y_STEP, initial_pos[1] + Y_STEP*np.sqrt(3), initial_pos[2] ]
    },
    'leg1_right': {
        'osc': 0,
        'P1' : [ initial_pos[0] - Y_STEP, initial_pos[1] + Y_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] + Y_STEP, initial_pos[1] - Y_STEP*np.sqrt(3), initial_pos[2] ]
    },
    # osc = 1
    'leg2_right': {
        'osc': 1,
        'P1' : [ initial_pos[0] - 2*Y_STEP, initial_pos[1], initial_pos[2] ],
        'P3' : [ initial_pos[0] + 2*Y_STEP, initial_pos[1], initial_pos[2] ]
    },
    'leg3_left':  {
        'osc': 1,
        'P1' : [ initial_pos[0] + Y_STEP, initial_pos[1] - Y_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] - Y_STEP, initial_pos[1] + Y_STEP*np.sqrt(3), initial_pos[2] ]
    },
    'leg1_left':  {
        'osc': 1,
        'P1' : [ initial_pos[0] + Y_STEP, initial_pos[1] + Y_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] - Y_STEP, initial_pos[1] - Y_STEP*np.sqrt(3), initial_pos[2] ]
    },
}

LEG_CONFIG_Sidle = {
    # osc = 0
    'leg2_left':  {
        'osc': 0,
        'P1' : [ initial_pos[0] - 2*X_STEP, initial_pos[1], initial_pos[2] ],
        'P3' : [ initial_pos[0] + 2*X_STEP, initial_pos[1], initial_pos[2] ]
    },
    'leg1_right': {
        'osc': 0,
        'P1' : [ initial_pos[0] + X_STEP, initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] - X_STEP, initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ]
    },
    'leg3_right': {
        'osc': 0,
        'P1' : [ initial_pos[0] + X_STEP, initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] - X_STEP, initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ]
    },
    # osc = 1
    'leg2_right': {
        'osc': 1,
        'P1' : [ initial_pos[0] + 2*X_STEP, initial_pos[1], initial_pos[2] ],
        'P3' : [ initial_pos[0] - 2*X_STEP, initial_pos[1], initial_pos[2] ]
    },
    'leg1_left':  {
        'osc': 1,
        'P1' : [ initial_pos[0] - X_STEP, initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] + X_STEP, initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ]
    },
    'leg3_left':  {
        'osc': 1,
        'P1' : [ initial_pos[0] - X_STEP, initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] + X_STEP, initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ]
    },
}

# LEG_CONFIG_Turn = {
#     # osc = 0
#     'leg2_left':  {
#         'osc': 0,
#         'P1' : [ initial_pos[0], initial_pos[1], initial_pos[2] ],
#         'P3' : 0.2
#     },
#     'leg1_right': {
#         'osc': 0,
#         'P1' : [ initial_pos[0], initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ],
#         'P3' : -0.2
#     },
#     'leg3_right': {
#         'osc': 0,
#         'P1' : [ initial_pos[0], initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ],
#         'P3' : -0.2
#     },
#     # osc = 1
#     'leg2_right': {
#         'osc': 1,
#         'P1' : [ initial_pos[0], initial_pos[1], initial_pos[2] ],
#         'P3' : -0.2
#     },
#     'leg1_left':  {
#         'osc': 1,
#         'P1' : [ initial_pos[0], initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ],
#         'P3' : 0.2
#     },
#     'leg3_left':  {
#         'osc': 1,
#         'P1' : [ initial_pos[0], initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ],
#         'P3' : 0.2
#     },
# }
# ——— 左右腿对应的符号 ———
# 用于将 IK 输出的角度转换到 Webots 里：右腿 +，左腿 –
leg_sign = {
    'left':  -1,
    'right': +1
}