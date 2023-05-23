import RPi.GPIO as GPIO
import time
from dvg_pid_controller import PID_Controller, Constants
import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as subscribe
import os

servoPIN = 4
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(servoPIN, GPIO.OUT)

def input():
    msg = subscribe.simple("bbq/temperature/1", hostname=mqtt_server_ip)
    temperature = float(msg.payload.decode())
    return temperature


mqtt_server_ip = os.environ['MQTT_SERVER']

if __name__ == '__main__':
    if mqtt_server_ip is not None:
        mqtt_client = mqtt.Client()
        mqtt_client.connect(mqtt_server_ip, 1883, 60)
        mqtt_client.loop_start()
    else:
        raise NameError("No MQTT Server configured")
#    msg1 = subscribe.simple("bbq/temperature/1", hostname=mqtt_server_ip)

    p = GPIO.PWM(servoPIN, 50)  # GPIO 4 for PWM with 50Hz
    # Initialization
    output = 4
    
    p.start(output)
    mqtt_client.publish("heater/command", str(output))

#    pid = PID_Controller(Kp=0.1, Ki=0.00001, Kd=0.0)
#    pid.set_tunings(Kp=0.1, Ki=0.00001, Kd=0.0)
#    pid.set_output_limits(2, 11)
#    pid.setpoint = 400
#    pid.set_mode(Constants.AUTOMATIC, current_input=input(), current_output=output)
    try:
        while True:
            #pid.compute(current_input=input())
            #output = pid.output
            #print("Updating Cmd")
            #msg = subscribe.simple("dashboard/command", hostname=mqtt_server_ip)
            #output = float(msg.payload.decode())
#            p.ChangeDutyCycle(output)
#            mqtt_client.publish("heater/command", str(output))
#            print("PID: Output is now: %s error: %s input: %s time: %s" % (output,pid.last_error,pid.last_input,pid.last_time))
#            msg = subscribe.simple("heater/command", hostname=mqtt_server_ip)
            msg = subscribe.simple("coffee/control/Command", hostname=mqtt_server_ip,retained=True)
            output = float(msg.payload.decode())
            print("Output is now: %s " % (output))

            p.ChangeDutyCycle(output)
            mqtt_client.publish("heater/command", str(output))
            time.sleep(1)
    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()
