"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""
import logging
from math import floor
from typing import List, Tuple

try:
    import RPi.GPIO as GPIO
    import wiringpi
except ModuleNotFoundError as e:
    pass

import time

import donkeycar as dk
from donkeycar.parts.part import Part

THROTTLE = 'throttle'

ANGLE = 'angle'

logger = logging.getLogger(__name__)


class PCA9685:
    '''
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    '''

    def __init__(self, channel, frequency=60):
        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse)

    def run(self, pulse):
        self.set_pulse(pulse)


class WiringPiPWM:
    """ Soft controller based on wiringpi library
    """

    def __init__(self, pin: int, pwm_range=100):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self._pin = pin

        GPIO.setup(self._pin, GPIO.OUT, initial=GPIO.LOW)

        wiringpi.wiringPiSetupGpio()
        wiringpi.softPwmCreate(self._pin, 0, pwm_range)

    def set_pulse(self, pulse):
        wiringpi.softPwmWrite(self._pin, pulse)

    def _stop(self):
        logger.debug('motor_stop')
        wiringpi.softPwmWrite(self._pin, 0)

    def shutdown(self):
        self._stop()
        GPIO.cleanup()


class PWMSteering(Part):
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """

    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                 left_pulse=290,
                 right_pulse=490):
        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse

    def run(self, angle):
        # map absolute angle to angle that vehicle can implement.
        pulse = dk.utils.map_range(angle,
                                   self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                   self.left_pulse, self.right_pulse)

        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0)  # set steering straight

    def get_inputs_keys(self) -> List[str]:
        return [ANGLE]

    def get_outputs_keys(self) -> List[Tuple[str, str]]:
        return []


class PWMThrottle(Part):
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """

    MIN_THROTTLE = -1
    MAX_THROTTLE = 1

    def __init__(self, controller=None,
                 max_pulse=300,
                 min_pulse=490,
                 zero_pulse=350):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        # send zero pulse to calibrate ESC
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)

    def run(self, throttle):
        if throttle > 0:
            pulse = dk.utils.map_range(throttle,
                                       0, self.MAX_THROTTLE,
                                       self.zero_pulse, self.max_pulse)
        else:
            pulse = dk.utils.map_range(throttle,
                                       self.MIN_THROTTLE, 0,
                                       self.min_pulse, self.zero_pulse)

        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0)  # stop vehicle

    def get_inputs_keys(self) -> List[str]:
        return [THROTTLE]

    def get_outputs_keys(self) -> List[Tuple[str, str]]:
        return []


class GpioMotor(Part):
    """
    Soft PWM control
    Used on a differential drive car.
    """

    def __init__(self, pwm_range=100):
        self.ENA = 13
        self.ENB = 20
        self.IN1 = 19
        self.IN2 = 16
        self.IN3 = 21
        self.IN4 = 26
        self.IR_M = 22

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.ENA, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ENB, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN4, GPIO.OUT, initial=GPIO.LOW)

        GPIO.setup(self.IR_M, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        wiringpi.wiringPiSetupGpio()
        wiringpi.softPwmCreate(self.IN1, 0, pwm_range)
        wiringpi.softPwmCreate(self.IN2, 0, pwm_range)
        wiringpi.softPwmCreate(self.IN3, 0, pwm_range)
        wiringpi.softPwmCreate(self.IN4, 0, pwm_range)
        self.max_pulse = pwm_range

    def run(self, throttle: float, angle=0.0):
        straight_pulse = self.max_pulse * abs(throttle)
        if angle < 0.0:
            left_pulse = floor(straight_pulse * (1 - abs(angle)))
            right_pulse = floor(straight_pulse)
        else:
            left_pulse = floor(straight_pulse)
            right_pulse = floor(straight_pulse * (1 - angle))

        logger.debug(
            "angle {0:>+4.2f} | throttle {1:>+4.2f} | L pulse {2:>4f} | R pulse {3:>4f}".format(angle, throttle,
                                                                                                left_pulse,
                                                                                                right_pulse))

        if throttle < 0:
            self._left_backward(left_pulse)
            self._right_backward(right_pulse)
        else:
            self._left_forward(left_pulse)
            self._right_forward(right_pulse)

    def _right_forward(self, pulse):
        GPIO.output(self.ENA, GPIO.HIGH)
        wiringpi.softPwmWrite(self.IN1, 0)
        wiringpi.softPwmWrite(self.IN2, pulse)

    def _right_backward(self, pulse):
        GPIO.output(self.ENA, GPIO.HIGH)
        wiringpi.softPwmWrite(self.IN2, 0)
        wiringpi.softPwmWrite(self.IN1, pulse)

    def _left_forward(self, pulse):
        GPIO.output(self.ENB, GPIO.HIGH)
        wiringpi.softPwmWrite(self.IN4, 0)
        wiringpi.softPwmWrite(self.IN3, pulse)

    def _left_backward(self, pulse):
        GPIO.output(self.ENB, GPIO.HIGH)
        wiringpi.softPwmWrite(self.IN3, 0)
        wiringpi.softPwmWrite(self.IN4, pulse)

    def _stop(self):
        logger.debug('motor_stop')
        GPIO.output(self.ENA, False)
        GPIO.output(self.ENB, False)
        wiringpi.softPwmWrite(self.IN1, 0)
        wiringpi.softPwmWrite(self.IN2, 0)
        wiringpi.softPwmWrite(self.IN3, 0)
        wiringpi.softPwmWrite(self.IN4, 0)

    def shutdown(self):
        self._stop()
        GPIO.cleanup()

    def get_inputs_keys(self) -> List[str]:
        return [THROTTLE, ANGLE]

    def get_outputs_keys(self) -> List[str]:
        return []


class Adafruit_DCMotor_Hat:
    '''
    Adafruit DC Motor Controller
    Used for each motor on a differential drive car.
    '''

    def __init__(self, motor_num):
        from Adafruit_MotorHAT import Adafruit_MotorHAT
        import atexit

        self.FORWARD = Adafruit_MotorHAT.FORWARD
        self.BACKWARD = Adafruit_MotorHAT.BACKWARD
        self.mh = Adafruit_MotorHAT(addr=0x60)

        self.motor = self.mh.getMotor(motor_num)
        self.motor_num = motor_num

        atexit.register(self.turn_off_motors)
        self.speed = 0
        self.throttle = 0

    def run(self, speed):
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed > 1 or speed < -1:
            raise ValueError("Speed must be between 1(forward) and -1(reverse)")

        self.speed = speed
        self.throttle = int(dk.utils.map_range(abs(speed), -1, 1, -255, 255))

        if speed > 0:
            self.motor.run(self.FORWARD)
        else:
            self.motor.run(self.BACKWARD)

        self.motor.setSpeed(self.throttle)

    def shutdown(self):
        self.mh.getMotor(self.motor_num).run(Adafruit_MotorHAT.RELEASE)


class Maestro:
    '''
    Pololu Maestro Servo controller
    Use the MaestroControlCenter to set the speed & acceleration values to 0!
    '''
    import threading

    maestro_device = None
    astar_device = None
    maestro_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency=60):
        import serial

        if Maestro.maestro_device == None:
            Maestro.maestro_device = serial.Serial('/dev/ttyACM0', 115200)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Maestro.astar_device == None:
            Maestro.astar_device = serial.Serial('/dev/ttyACM2', 115200, timeout=0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096))  # in seconds
        w *= 1000 * 1000  # in microseconds
        w *= 4  # in quarter microsenconds the maestro wants
        w = int(w)

        with Maestro.maestro_lock:
            Maestro.maestro_device.write(bytearray([0x84,
                                                    self.channel,
                                                    (w & 0x7F),
                                                    ((w >> 7) & 0x7F)]))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def readline(self):
        ret = None
        with Maestro.astar_lock:
            # expecting lines like
            # E n nnn n
            if Maestro.astar_device.inWaiting() > 8:
                ret = Maestro.astar_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret


class Teensy:
    '''
    Teensy Servo controller
    '''
    import threading

    teensy_device = None
    astar_device = None
    teensy_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency=60):
        import serial

        if Teensy.teensy_device == None:
            Teensy.teensy_device = serial.Serial('/dev/teensy', 115200, timeout=0.01)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Teensy.astar_device == None:
            Teensy.astar_device = serial.Serial('/dev/astar', 115200, timeout=0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096))  # in seconds
        w *= 1000 * 1000  # in microseconds

        with Teensy.teensy_lock:
            Teensy.teensy_device.write(("%c %.1f\n" % (self.channel, w)).encode('ascii'))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def teensy_readline(self):
        ret = None
        with Teensy.teensy_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.teensy_device.inWaiting() > 8:
                ret = Teensy.teensy_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

    def astar_readline(self):
        ret = None
        with Teensy.astar_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.astar_device.inWaiting() > 8:
                ret = Teensy.astar_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret


class MockController(object):
    def __init__(self):
        pass

    def run(self, pulse):
        pass

    def shutdown(self):
        pass
