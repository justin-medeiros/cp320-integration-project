import RPi.GPIO as GPIO
import time
import threading
import smbus
from datetime import datetime
import os

# Input devices
MOTOR = 0
LED = 1
INPUT_DEVICES = [MOTOR, LED]

# Output devices
DIST_SENSOR = 0
POTENTIOMETER = 1
OUTPUT_DEVICES = [DIST_SENSOR, POTENTIOMETER]

# Motor constants
FULL_ROTATION_STEPS = 4096
FULL_ROTATION_DEGREES = 360
SLEEP_TIME = 0.005

# Led constants
FREQUENCY = 50

# Potentiometer constants
ADDRESS = 0x48
POTENTIOMETER_SENSOR = 0x43
DUTY_CYCLE_VAL = 255

# Distance sensor constants
HIGH_TIME=0.000001
LOW_TIME=1-HIGH_TIME
SPEED_OF_SOUND=330/float(1000000)
MAX_DIST=0.4
MIN_DIST=0.035
MAX_SLEEP_TIME = 0.05

"""
    This class is used to control the pi.
    It contains methods to read the distance sensor, potentiometer, and operate the motor and led.
"""

class Controller:
    def __init__(self, dist_trigger_pin, dist_echo_pin, led_pin, motor_pins):
        all_pins = [dist_trigger_pin, dist_echo_pin, led_pin] + motor_pins
        assert len(all_pins) == len(set(all_pins)), "All pins must be unique."

        # Initialize pins
        self.dist_trigger_pin = dist_trigger_pin
        self.dist_echo_pin = dist_echo_pin
        self.motor_pins = motor_pins
        self.led_pin = led_pin

        # 1 = potent -> led, dist -> motor
        # 2 = potent -> motor, dist -> led
        self.in_to_out_lock = threading.Lock() # Lock to change input-to-output mapping
        self.in_to_out = 1

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dist_trigger_pin, GPIO.OUT)
        GPIO.setup(self.dist_echo_pin, GPIO.IN)
        GPIO.setup(self.led_pin, GPIO.OUT)
        GPIO.setup(self.motor_pins, GPIO.OUT)

        # Initialize stepper sequence
        self.stepper_sequence=[
            [GPIO.HIGH, GPIO.LOW, GPIO.LOW,GPIO.LOW], # 1000
            [GPIO.HIGH, GPIO.HIGH, GPIO.LOW,GPIO.LOW], # 1100
            [GPIO.LOW, GPIO.HIGH, GPIO.LOW,GPIO.LOW], # 0100
            [GPIO.LOW, GPIO.HIGH, GPIO.HIGH,GPIO.LOW], # 0110
            [GPIO.LOW, GPIO.LOW, GPIO.HIGH,GPIO.LOW], # 0010
            [GPIO.LOW, GPIO.LOW, GPIO.HIGH,GPIO.HIGH], # 0011
            [GPIO.LOW, GPIO.LOW, GPIO.LOW,GPIO.HIGH], # 0001
            [GPIO.HIGH, GPIO.LOW, GPIO.HIGH,GPIO.HIGH] # 1001
        ]

        # Thread variables
        self.stop = False
        self.stop_lock = threading.Lock() # Lock to stop threads

        # Stepper motor variables
        self.last_step_idx = 0 # Last step in stepper sequence taken
        self.step_count = 0 # Total number of steps taken
        self.motor_sleep = SLEEP_TIME # Sleep time between steps
        self.motor_sleep_lock = threading.Lock() # Lock to change sleep time

        # User input variables
        self.user_input = None # User input
        self.user_input_lock = threading.Lock() # Lock to get user input

        # Led variables
        self.led_on = True # ! Potential dont Flag to turn led on/off
        self.pwn_led = GPIO.PWM(led_pin, FREQUENCY) # PWM object for led
        self.led_duty_cycle = None
        self.led_duty_cycle_lock = threading.Lock() # Lock to change led duty cycle

        # Potentiometer variables
        self.bus = smbus.SMBus(1)
        self.potentiometer_address = ADDRESS
        self.potentiometer_register = POTENTIOMETER_SENSOR

        # Distance sensor variables
        self.distance = None
        self.distance_lock = threading.Lock() # Lock to change distance

    def run(self):
        # Start threads to get user input at any time
        user_input_thread = threading.Thread(target=self.get_user_input)
        user_input_thread.start()

        motor_thread = threading.Thread(target=self.operate_motor)
        motor_thread.start()

        led_thread = threading.Thread(target=self.operate_led)
        led_thread.start()

        dist_thread = threading.Thread(target=self.read_distance_sensor)
        dist_thread.start()

        try:
            while True:
                if self.check_input() is False:
                    break

                # Get input values from sensors
                distance_value = self.get_dist_reading()
                potentiometer_value = self.read_potentiometer()

                # Set output values based on self.in_to_out
                sleep_time = None
                led_duty_cycle = None
                if self.in_to_out == 1:
                    sleep_time = self.calc_dist_sleep_time(distance_value)
                    led_duty_cycle = self.calc_potent_duty_cycle(potentiometer_value)
                else:
                    sleep_time = self.calc_potent_sleep_time(potentiometer_value)
                    led_duty_cycle = self.calc_dist_duty_cycle(distance_value)

                # Set values
                self.set_lock(self.motor_sleep_lock, 'motor_sleep', sleep_time)
                self.set_lock(self.led_duty_cycle_lock, 'led_duty_cycle', led_duty_cycle)

                # Sleep
                time.sleep(0.5)

            user_input_thread.join()
            motor_thread.join()
            dist_thread.join()
            led_thread.join()
        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Exiting...")
        finally:
            self.cleanup()
            os._exit(0)


    def get_user_input(self):
        while True:
            if self.read_lock(self.stop_lock, 'stop'):
                break

            if self.read_lock(self.user_input_lock, 'user_input') is not None:
                continue

            selected_option = self.read_lock(self.in_to_out_lock, 'in_to_out')

            # Modify the prompt based on the selected option
            prompt_lines = [
                'These are the options you can select',
                '1. Potentiometer controls LED brightness and distance sensor controls motor',
                '2. Distance sensor controls LED brightness and potentiometer controls motor',
                'q. Quit',
                '-------------------------------------'
            ]
            if selected_option == 1:
                prompt_lines[1] = prompt_lines[1] + '(SELECTED)'
            elif selected_option == 2:
                prompt_lines[2] = prompt_lines[2] + '(SELECTED)'

            # Get user input
            user_input = input('\n'.join(prompt_lines) + '\nSelect an option: ')
            self.set_lock(self.user_input_lock, 'user_input', user_input)

            if user_input == "q":
                break
        
    """ 
        Checks what user has been inputting
        returns False if user has inputted 'q'
    """
    def check_input(self):
        with self.user_input_lock:
            with self.in_to_out_lock:
                user_input = self.user_input
                # Break out early if no input has been given
                if user_input is None:
                    return True

                in_to_out = self.in_to_out

                if user_input == "q":
                    self.set_lock(self.stop_lock, 'stop', True)
                    print("Quitting...")
                    return False
                else:
                    try:
                        user_input = int(user_input)
                        if user_input == in_to_out:
                            print("Output already set to {}".format(user_input))
                        elif user_input == 1 or user_input == 2:
                            self.in_to_out = user_input
                            print("Switching output...")
                    except ValueError:
                        print("Invalid input")

                # Need to reset user input to None
                self.user_input = None
                return True

    def get_dist_reading(self):
        distance = self.read_lock(self.distance_lock, 'distance')
        return distance

    def read_distance_sensor(self):
        while True:
            if self.read_lock(self.stop_lock, 'stop'):
                break

            GPIO.output(self.dist_trigger_pin, GPIO.HIGH)
            time.sleep(HIGH_TIME)
            GPIO.output(self.dist_trigger_pin, GPIO.LOW)

            while GPIO.input(self.dist_echo_pin) == False:
                pass
            pulse_start = datetime.now().microsecond
            while GPIO.input(self.dist_echo_pin) == True:
                pass
            pulse_end = datetime.now().microsecond

            pulse_duration = pulse_end - pulse_start
            distance = self.get_distance(pulse_duration)
            # Update distance
            self.set_lock(self.distance_lock, 'distance', distance)
            time.sleep(LOW_TIME)

    def read_potentiometer(self):
        self.bus.write_byte(self.potentiometer_address, self.potentiometer_register)
        return self.bus.read_byte(self.potentiometer_address)

    """
        Turn the motor based on the angle given
    """
    def operate_motor(self):
        i = 0
        try:
            while True:
                if self.read_lock(self.stop_lock, 'stop'):
                    break
                sleep_time = self.read_lock(self.motor_sleep_lock, 'motor_sleep')
                if sleep_time == None:
                    continue

                GPIO.output(self.motor_pins, self.stepper_sequence[i])
                self.step_count += 1
                time.sleep(sleep_time)
                self.last_step_idx = i
                i+=1

                if i > len(self.stepper_sequence) - 1:
                    i = 0

        except KeyboardInterrupt:
            print("Broke out while rotating")
            self.cleanup()

    def operate_led(self):
        try:

            # Initial reading of the start value for the PWM
            value = self.read_potentiometer()

            # Set the initial value of the duty cycle
            start_duty_cycle = self.calc_potent_duty_cycle(value)
            self.set_lock(self.led_duty_cycle_lock, 'led_duty_cycle', start_duty_cycle)

            # Set the start value for the PWM and start the duty cycle
            self.pwn_led.start(start_duty_cycle)

            while True:
                if self.read_lock(self.stop_lock, 'stop'):
                    break
                duty_cycle = self.read_lock(self.led_duty_cycle_lock, 'led_duty_cycle')
                if duty_cycle == None:
                    continue
                self.pwn_led.ChangeDutyCycle(duty_cycle)
        except KeyboardInterrupt:
            print("Broke out while changing LED brightness")
            self.cleanup()

    """
        This method is used to reset the motor to its original position.
    """
    def reset_motor(self):
        print('Resetting motor...')
        steps_taken = self.step_count % FULL_ROTATION_STEPS
        reverse_steps = self.stepper_sequence[::-1]
        i = 0
        try:
            while i < steps_taken:
                i += 1
                GPIO.output(self.motor_pins, reverse_steps[(self.last_step_idx + i) % len(self.stepper_sequence)])
                time.sleep(SLEEP_TIME)
            self.last_step_idx = 0
            self.step_count = 0
        except KeyboardInterrupt:
            print("Broke out while returning to null")

    """
        This method is used to toggle the lock on a given flag
        lock: the lock to acquire
        flag: the flag to toggle
        value: the value to set the flag to
    """
    def set_lock(self, lock, flag, value):
        with lock:
            setattr(self, flag, value)

    def read_lock(self, lock, flag):
        with lock:
            return getattr(self, flag)

    def calc_potent_duty_cycle(self, potent_value):
        return (potent_value / DUTY_CYCLE_VAL) * 100

    def calc_dist_duty_cycle(self, dist):
        if dist is None:
            return None

        dist = max(MIN_DIST, min(MAX_DIST, dist))
        percent = (dist - MIN_DIST) / (MAX_DIST - MIN_DIST)
        duty_cycle = 100 - percent * 100
        return duty_cycle

    def calc_potent_sleep_time(self, val):
        min_sleep_time = SLEEP_TIME
        return min_sleep_time + (MAX_SLEEP_TIME - min_sleep_time) * (val / DUTY_CYCLE_VAL)

    def calc_dist_sleep_time(self, distance):
        if distance is None:
            return None

        dist = max(MIN_DIST, min(MAX_DIST, distance))

        percent = (dist - MIN_DIST) / (MAX_DIST - MIN_DIST)

        sleep_time = SLEEP_TIME + percent * (MAX_SLEEP_TIME - SLEEP_TIME)
        return sleep_time

    def cleanup(self):
        self.reset_motor()
        self.pwn_led.stop()
        GPIO.cleanup()
        print("Finished cleaning up")
        return

    def get_distance(self, td):
        distance=SPEED_OF_SOUND*td/float(2)
        return distance
