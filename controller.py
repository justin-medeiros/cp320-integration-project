import RPi.GPIO as GPIO
import time
import threading
import smbus

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

# Distance sensor constants
HIGH_TIME=0.000001
LOW_TIME=1-HIGH_TIME
SPEED_OF_SOUND=330/float(1000000)

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
        self.input_to_output = {
            DIST_SENSOR: MOTOR,
            POTENTIOMETER: LED
        }

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        """ GPIO.setup(self.dist_trigger_pin, GPIO.OUT)"""
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
        
        # Stepper motor variables
        self.last_step_idx = 0 # Last step in stepper sequence taken
        self.step_count = 0 # Total number of steps taken
        self.stop_motor = False # Flag to stop motor
        self.stop_motor_lock = threading.Lock() # Lock for stop motor flag
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
    
    def switch_output(self):
        # Swap input-to-output mappings
        for input_device, output_device in self.input_to_output.items():
            self.input_to_output[input_device] = (output_device + 1) % len(self.OUTPUT_DEVICES)


    def run(self):
        # Start thread to get user input at any time
        user_input_thread = threading.Thread(target=self.get_user_input)
        user_input_thread.start()

        # motor_thread = threading.Thread(target=self.operate_motor)
        # motor_thread.start()

        led_thread = threading.Thread(target=self.operate_led)
        led_thread.start() 

        try:
            while True:
                with self.user_input_lock:
                    user_input = self.user_input

                if user_input == "q":
                    break
                elif user_input == "s":
                    print("DO THIS SWITCGH")
                    # self.switch_output()

                # Get input values from sensors
                # distance_value = self.read_distance_sensor()
                potentiometer_value = self.read_potentiometer()

                # Set output values based on self.input_to_output
                # sleep_time = None

                if True: # * Change to this -> (self.input_to_output[DIST_SENSOR] == MOTOR)
                    # sleep_time = self.calc_dist_sleep_time(distance_value)
                    # print(sleep_time)
                    led_duty_cycle = self.calc_potent_duty_cycle(potentiometer_value)
                else:
                    pass
                    #sleep_time = self.calc_potent_sleep_time(potentiometer_value)
                    #led_duty_cycle = self.calc_dist_duty_cycle(distance_value)

                # Set values
                #self.set_lock(self.motor_sleep_lock, 'motor_sleep', sleep_time)
                self.set_lock(self.led_duty_cycle_lock, 'led_duty_cycle', led_duty_cycle)

                # Sleep
                time.sleep(0.1)

            """ user_input_thread.join()
            motor_thread.join()
            led_thread.join() """
            
        except KeyboardInterrupt:
            pass
        finally:
            print('hello clean up')
            self.cleanup()
        return


    def get_user_input(self):
        while True:
            # Get user input
            user_input = input("Enter input: ")
            self.set_lock(self.user_input_lock, 'user_input', user_input)
            if(user_input=='q'):
                break
        
    def read_distance_sensor(self):
        GPIO.output(self.dist_trigger_pin, GPIO.HIGH)
        time.sleep(HIGH_TIME)
        GPIO.output(self.dist_trigger_pin, GPIO.LOW)

        pulse_start = time.time()
        pulse_end = time.time()

        while GPIO.input(self.dist_echo_pin) == 0:
            pulse_start = time.time()

        while GPIO.input(self.dist_echo_pin) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        return self.get_distance(pulse_duration)


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
                if self.read_lock(self.stop_motor_lock, 'stop_motor'):
                    break

                GPIO.output(self.motor_pins, self.stepper_sequence[i])
                self.step_count += 1
                time.sleep(self.read_lock(self.motor_sleep_lock, 'motor_sleep'))
                self.last_step_idx = i
                i+=1

                if i > 7:
                    i = 0

        except KeyboardInterrupt:
            print("Broke out while rotating")

    def operate_led(self):
        prev_value = None
        
        try:
            
            # Initial reading of the start value for the PWM
            value = self.read_potentiometer()

            # Set the previous value
            prev_value = value
            
            # Set the initial value of the duty cycle
            self.set_lock(self.led_duty_cycle_lock, 'led_duty_cycle', value)

            # Set the start value for the PWM and start the duty cycle
            start_duty_cycle = self.calc_potent_duty_cycle(value)
            self.pwn_led.start(start_duty_cycle) 
            print("START:%1.3f  " %(start_duty_cycle))

            while True:
                # Read value from

                value = self.read_lock(self.led_duty_cycle_lock, 'led_duty_cycle')
                if value != prev_value:
                    duty_cycle = self.calc_potent_duty_cycle(value)
                    self.pwn_led.ChangeDutyCycle(duty_cycle)
                    print('duty cycle: ', duty_cycle)
                    prev_value = value
                time.sleep(0.1)
                    
           
        except KeyboardInterrupt:
            pass


    """ 
        This method is used to reset the motor to its original position.
    """
    def reset_motor(self):
        steps_taken = self.step_count % FULL_ROTATION_STEPS
        reverse_steps = self.stepper_sequence[::-1]
        i = 0
        try:
            while i < steps_taken:
                i += 1
                GPIO.output(self.motor_pins, reverse_steps[(self.last_step_idx + i) % 8])
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
        return (potent_value / 255.0) * 100

    def calc_dist_duty_cycle(self, val):
        pass

    def calc_potent_sleep_time(self, val):
        min_sleep_time = SLEEP_TIME
        max_sleep_time = 0.05
        return min_sleep_time + (max_sleep_time - min_sleep_time) * (val / 255.0)

    def calc_dist_sleep_time(self, distance):
        print(distance)
        if distance > 10:
            return 0.05
        elif distance > 5:
            return 0.01
        else:
            return 0.005

    def cleanup(self):
        self.reset_motor()
        self.pwn_led.stop()
        GPIO.cleanup()
        return

    def get_distance(td):
        distance=SPEED_OF_SOUND*td/float(2)
        return distance
