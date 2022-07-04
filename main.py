import RPi.GPIO as GPIO
import time
from dvg_pid_controller import PID_Controller, Constants

servoPIN = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

def update():

if __name__ == '__main__':
    p = GPIO.PWM(servoPIN, 50)  # GPIO 4 for PWM with 50Hz
    output = 6  # Initialization
    p.start(output)
    pid = PID_Controller()
    pid.set_tunings(Kp=0.1, Ki=0.004, Kd=0.0)
    pid.set_output_limits(2, 11)
    pid.setpoint=390
    pid.set_mode(Constants.AUTOMATIC, current_input=update(), current_output=output)
    try:
        while True:
            pid.compute(current_input=update())
            p.ChangeDutyCycle(pid.output)
            time.sleep(1)
    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()

