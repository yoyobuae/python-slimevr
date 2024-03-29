PACKET_HEARTBEAT = 0
# PACKET_ROTATION = 1 # Deprecated
# PACKET_GYRO = 2 # Deprecated
PACKET_HANDSHAKE = 3
PACKET_ACCEL = 4
# PACKET_MAG = 5 # Deprecated
# PACKET_RAW_CALIBRATION_DATA = 6 # Deprecated
# PACKET_CALIBRATION_FINISHED = 7 # Deprecated
PACKET_CONFIG = 8
# PACKET_RAW_MAGNETOMETER = 9 # Deprecated
PACKET_PING_PONG = 10
PACKET_SERIAL = 11
PACKET_BATTERY_LEVEL = 12
PACKET_TAP = 13
PACKET_ERROR = 14
PACKET_SENSOR_INFO = 15
# PACKET_ROTATION_2 = 16 # Deprecated
PACKET_ROTATION_DATA = 17
PACKET_MAGNETOMETER_ACCURACY = 18
PACKET_SIGNAL_STRENGTH = 19
PACKET_TEMPERATURE = 20
# PACKET_USER_ACTION = 21 # Joycon buttons only currently
PACKET_FEATURE_FLAGS = 22

PACKET_BUNDLE = 100

PACKET_INSPECTION = 105  # 0x69

PACKET_RECEIVE_HEARTBEAT = 1
PACKET_RECEIVE_VIBRATE = 2
PACKET_RECEIVE_HANDSHAKE = 3
PACKET_RECEIVE_COMMAND = 4

PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA = 1
PACKET_INSPECTION_PACKETTYPE_FUSED_IMU_DATA = 2
PACKET_INSPECTION_PACKETTYPE_CORRECTION_DATA = 3
PACKET_INSPECTION_DATATYPE_INT = 1
PACKET_INSPECTION_DATATYPE_FLOAT = 2

BOARD_UNKNOWN = 0

IMU_UNKNOWN = 0
IMU_MPU9250 = 1
IMU_MPU6500 = 2
IMU_BNO080 = 3
IMU_BNO085 = 4
IMU_BNO055 = 5
IMU_MPU6050 = 6
IMU_BNO086 = 7
IMU_BMI160 = 8
IMU_ICM20948 = 9
IMU_ICM42688 = 10
IMU_DEV_RESERVED = 250 # Reserved, should not be used in any release firmware

MCU_UNKNOWN = 0

MAC = bytes('python', 'utf-8')

FIRMWARE_BUILD_NUMBER = 17
FIRMWARE_VERSION = b'0.4.0'

TIMEOUT = 3000

MAX_IMU_COUNT = 1

DATA_TYPE_NORMAL = 1
DATA_TYPE_CORRECTION = 2
