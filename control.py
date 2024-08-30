import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import math
import argparse

import time

class Control:
    def __init__(self) -> None:
        pass

class Stepper:
    def __init__(self, GPIO_pins = (14, 15, 18), direction = 20, step = 21):
        self.myStepper = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "A4988")
        self.distance = 0
        
    # Function to calculate steps needed for a given distance
    def calculate_steps(self, distance, wheel_radius=0.02, steps_per_revolution=200, ratio = 32):
        circumference = 2 * math.pi * wheel_radius
        revolutions = distance / circumference
        steps = int(revolutions * steps_per_revolution * ratio)
        return steps

    # Function to move the motor clockwise for a given distance
    def move_clockwise(self, distance): # distance in meters
        steps = self.calculate_steps(distance)
        self.myStepper.motor_go(True, "Full", steps, .00025, True, .05)

    # Function to move the motor counterclockwise for a given distance
    def move_counterclockwise(self, distance): # distance in meters
        steps = self.calculate_steps(distance)
        self.myStepper.motor_go(False, "Full", steps, .00025, True, .05)
        
    def run_stepper(self, distance, direction):
        if direction == "forward":
            self.move_counterclockwise(distance)
            # self.distance += distance
        elif direction == "backward":
            self.move_clockwise(distance)
            # self.distance -= distance
        else:
            print("Invalid direction. Use 'forward' or 'backward'.")

class Servo:
    def __init__(self, servo_pin = 18, start_angle = 35):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.servo_pin = servo_pin
        GPIO.setup(self.servo_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(start_angle)
    
    def set_servo_angle(self, angle):
        """Sets the servo to the specified angle."""
        # Map angle to duty cycle
        duty_cycle = 2.5 + (angle / 18.0)  # Calculate duty cycle from angle
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(1)  # Allow time for the servo to move
    
    def open_handle(self):
        """open the servo."""
        self.set_servo_angle(60)

    def close_handle(self):
        """close the servo."""
        self.set_servo_angle(35)
    
    def run_servo(self, direction):
        if direction == "close":
            self.close_handle()
        elif direction == "open":
            self.open_handle()
        else:
            print("Invalid direction. Use 'close' or 'open'.")
    
    def servo_clean(self):
        GPIO.cleanup()
    
class Buzzer:
    def __init__(self, buzzer_pin=18):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.buzzer_pin = buzzer_pin
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
    
    def buzz(self, duration=1):
        """Activates the buzzer for the specified duration."""
        GPIO.output(self.buzzer_pin, GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(self.buzzer_pin, GPIO.LOW)
    
    def buzz_pattern(self, pattern):
        """
        Activates the buzzer based on a pattern.
        :param pattern: List of tuples with (duration, pause) for buzzing and silence.
        """
        for duration, pause in pattern:
            self.buzz(duration)
            time.sleep(pause)
    
    def buzzer_clean(self):
        """Cleans up the GPIO settings."""
        GPIO.cleanup()

# if __name__ == "__main__":
#     # Argument parser for command-line arguments
#     parser = argparse.ArgumentParser(description="Control stepper motor with distance and direction input.")
#     parser.add_argument("direction", type=str, choices=["open", "close"], help="Direction to move the servo motor (open or close).")
    
#     # Parse the command-line arguments
#     args = parser.parse_args()
    
#     servo = Servo()

#     # try:
#     print(f"{args}")
#     servo.run_servo(direction = args.direction)
#     # except:
#     #     servo.servo_clean()
#     # finally:
#     #     servo.servo_clean()

# if __name__ == "__main__":
#     servo = Servo()
#     try:
#         while 1:
#             d = input("sssss: ")
#             if d == "q":
#                 servo.open_handle()
#             if d =="w":
#                 servo.close_handle()
#     except:
#         pass
#     GPIO.cleanup()

# if __name__ == "__main__":
#     buzzer = Buzzer(buzzer_pin=21)
    
#     try:
#         buzzer.buzz(2)  # Buzz for 2 seconds
#         while 1:
#             buzzer.buzz_pattern([(0.1, 0.1),(0.1, 0.2),(0.1, 0.3),(0.1, 0.4),(0.1, 0.5),(0.1, 0.6)])  # Buzz pattern
        
#     except KeyboardInterrupt:
#         print("Program interrupted")
    
#     finally:
#         buzzer.buzzer_clean()
