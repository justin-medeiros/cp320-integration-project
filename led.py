import smbus
import RPi.GPIO as GPIO
import time

# Function to calculate the duty cycle from the potentiometer value
def calculate_duty_cycle(value):
    return (value / 255.0) * 100
    

def potentiometer_sensor_change():
    FREQUENCY = 50
    LED_PIN_NUMBER_BCM = 23
    ADDRESS = 0x48
    POTENTIOMETER_SENSOR = 0x43
    bus = smbus.SMBus(1)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_PIN_NUMBER_BCM, GPIO.OUT)

    # Setup PWM for LED
    pwm_led = GPIO.PWM(LED_PIN_NUMBER_BCM, FREQUENCY) 

    # Used to keep track of the previous value
    prev_value = None

    try:
        # Initial reading and initialize start value for PWM
        bus.write_byte(ADDRESS,POTENTIOMETER_SENSOR)
        value = bus.read_byte(ADDRESS)
        # Set the previous value
        prev_value = value
        # Set the start value for the PWM
        start_duty_cycle = calculate_duty_cycle(value)
        pwm_led.start(start_duty_cycle) 
        print("AOUT:%1.3f  " %(start_duty_cycle))
        while True:
            
            bus.write_byte(ADDRESS,POTENTIOMETER_SENSOR)
            value = bus.read_byte(ADDRESS)
            if(prev_value != value):
                duty_cycle = calculate_duty_cycle(value)
                print(duty_cycle)
                pwm_led.ChangeDutyCycle(duty_cycle)
                prev_value = value
            
            # print("AOUT:%1.3f  " %(value))
            
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        pwm_led.stop()
        GPIO.cleanup()
    return



potentiometer_sensor_change()