from controller import Controller

MOTOR_PINS = [21, 20, 26, 19]
TRIG_PIN = 18
ECHO_PIN = 16
#LED_PIN = 


def main():
    # Initialize the controller
    controller = Controller(18, 27, 17, MOTOR_PINS)

    controller.run()
    return

if __name__ == "__main__":
    main()