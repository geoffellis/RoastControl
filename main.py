import RPi.GPIO as GPIO
import time
from dvg_pid_controller import PID_Controller, Constants
import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as subscribe

servoPIN = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)


def value_from(msg1):
    temperature_1 = msg1.payload.decode()
    return temperature_1


mqtt_server_ip = os.environ['MQTT_SERVER']

if __name__ == '__main__':
    if mqtt_server_ip is not None:
        mqtt_client = mqtt.Client()
        mqtt_client.connect(mqtt_server_ip, 1883, 60)
        mqtt_client.loop_start()
    else:
        raise NameError("No MQTT Server configured")
    msg1 = subscribe.simple("bbq/temperature/1", hostname=mqtt_server_ip)

    p = GPIO.PWM(servoPIN, 50)  # GPIO 4 for PWM with 50Hz
    output = 6  # Initialization
    p.start(output)
    pid = PID_Controller()
    pid.set_tunings(Kp=0.1, Ki=0.004, Kd=0.0)
    pid.set_output_limits(2, 11)
    pid.setpoint = 390
    pid.set_mode(Constants.AUTOMATIC, current_input=value_from(msg1), current_output=output)
    try:
        while True:
            pid.compute(current_input=value_from(msg1))
            p.ChangeDutyCycle(pid.output)
            mqtt_client.publish("heater/servo/command", pid.output)
            time.sleep(1)
    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()
