"""Wiring from server/move.py, servo.py, ultra.py, LED.py and OLED.py."""

MOTOR_A_EN = 4
MOTOR_B_EN = 17
MOTOR_A_PIN1 = 26
MOTOR_A_PIN2 = 21
MOTOR_B_PIN1 = 27
MOTOR_B_PIN2 = 18
DIR_FORWARD = 0
DIR_BACKWARD = 1
LEFT_FORWARD = 1
LEFT_BACKWARD = 0
RIGHT_FORWARD = 0
RIGHT_BACKWARD = 1

# move.py's manual example uses 60; its move() default radius is 0.6.
DEFAULT_SPEED = 60
DEFAULT_TURN_RADIUS = 0.6
SERVO_ADDRESS = 0x40
SERVO_BUS = 1
SERVO_FREQUENCY = 50
SERVO_INITIAL = 300
SERVO_RANGES = ((150, 450), (160, 480), (100, 500), (100, 500), (100, 500))
SERVO_DIRECTIONS = (1, 0, 1, 1, 0)
ULTRASONIC_TRIGGER = 11
ULTRASONIC_ECHO = 8
# A bounded API policy, not a legacy calibration value (about 5.1 m).
ULTRASONIC_TIMEOUT = 0.03
