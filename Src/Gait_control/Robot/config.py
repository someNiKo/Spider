# unit: mm
COXA_LENGTH = 30.1
FEMUR_LENGTH = 60.91
TIBIA_LENGTH = 152.32

LINK_GROUND_LENGTH = 35.94
LINK_CRANK_LENGTH = 22.36        # Input link
LINK_COUPLER_LENGTH = 24.0       # Connection link
LINK_ROCKER_LENGTH = 27.0       # Output link


# unit: degree
DEFAULT_COXA_ANGLE = 90.0
DEFAULT_FEMUR_ANGLE = 90.0
DEFAULT_TIBIA_ANGLE = 0.0

DEFAULT_COXA_SERVO_OUTPUT = 90.0
DEFAULT_FEMUR_SERVO_OUTPUT = 90.0
DEFAULT_TIBIA_SERVO_OUTPUT = 90.0

MAX_COXA_SERVO_OUTPUT = {"left": 180.0, "right": 180.0}          # 通过试验来确定
MIN_COXA_SERVO_OUTPUT = {"left": 0.0, "right": 0.0}
MAX_FEMUR_SERVO_OUTPUT = {"left": 180.0, "right": 180.0}          
MIN_FEMUR_SERVO_OUTPUT = {"left": 0.0, "right": 0.0}
MAX_TIBIA_SERVO_OUTPUT = {"left": 180.0, "right": 180.0}         #用自身的值比较舵机的极限位置
MIN_TIBIA_SERVO_OUTPUT = {"left": 0.0, "right": 0.0}

