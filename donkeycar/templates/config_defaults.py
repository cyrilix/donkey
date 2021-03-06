"""
CAR CONFIG

This file is read by your car application's manage.py script to change the car
performance.

EXAMPLE
-----------
import dk
cfg = dk.load_config(config_path='~/d2/config.py')
print(cfg.CAMERA_RESOLUTION)

"""

import os

# PATHS
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')
MODELS_PATH = os.path.join(CAR_PATH, 'models')

# VEHICLE
DRIVE_LOOP_HZ = 20
MAX_LOOPS = 100000

# CAMERA
CAMERA_RESOLUTION = (128, 160)  # (height, width)
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

# ARDUINO
ARDUINO_SERIAL_PORT = '/dev/serial0'
ARDUINO_SERIAL_BAUDRATE = 115200

# STEERING
STEERING_CHANNEL = 0
# See http://wiringpi.com/pins/ for pin mapping
STEERING_PIN = 0
TX_STEERING_MIN = 1060
TX_STEERING_MAX = 1960
STEERING_LEFT_PWM = ((TX_STEERING_MAX) / (16666 / 4095))
STEERING_RIGHT_PWM = ((TX_STEERING_MIN) / (16666 / 4095))

############
# THROTTLE #
############
THROTTLE_CHANNEL = 1
# See http://wiringpi.com/pins/ for pin mapping
THROTTLE_PIN = 2
THROTTLE_FORWARD_PWM = 400
THROTTLE_STOPPED_PWM = 360
THROTTLE_REVERSE_PWM = 310

THROTTLE_STEERING_ENABLE = True
# Max speed between 0-1
THROTTLE_MIN_SPEED = 0.5
THROTTLE_MAX_SPEED = 1.0
THROTTLE_SAFE_ANGLE = 0.3
THROTTLE_DANGEROUS_ANGLE = 0.8

THROTTLE_STOP_ON_SHOCK = False

# TRAINING
BATCH_SIZE = 128
TRAIN_TEST_SPLIT = 0.8

# JOYSTICK
USE_JOYSTICK_AS_DEFAULT = False
JOYSTICK_MAX_THROTTLE = 0.25
JOYSTICK_STEERING_SCALE = 1.0
AUTO_RECORD_ON_THROTTLE = True

######################
# Contours detection #
######################
POLY_DP_MIN = 4
POLY_DP_MAX = 1000
ARC_LENGTH_MIN = 15
ARC_LENGTH_MAX = 100000000

##############
#  Threshold #
##############
#  Static implementation
THRESHOLD_LIMIT_MIN = 180
THRESHOLD_LIMIT_MAX = 230

#  Dynamic
THRESHOLD_DYNAMIC_ENABLE = True
THRESHOLD_DYNAMIC_INIT = 180
THRESHOLD_DYNAMIC_DELTA = 10

THRESHOLD_HORIZON = 0.0

# PILOT
DEBUG_PILOT = True

##########
#  Angle #
##########
USE_ONLY_NEAR_CONTOUR = True
NB_CONTOURS_TO_USE = 1
OUT_ZONE_PERCENT = 20
CENTRAL_ZONE_PERCENT = 20

#ANGLE_ALGO = 'opencv'
ANGLE_ALGO = 'keras'
KERAS_MODEL = '/home/pi/model_cat'

########
# MQTT #
########
MQTT_ENABLE = True
MQTT_PUBLISH_ALL_EVENTS = True
MQTT_HOSTNAME = 'localhost'
MQTT_PORT = 1883
MQTT_QOS = 0
