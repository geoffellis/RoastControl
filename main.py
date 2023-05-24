import os
import time
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as subscribe

# GPIO pin for PWM
PWM_PIN = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)

mqtt_server_ip = os.environ.get('MQTT_SERVER')

if __name__ == '__main__':
    if mqtt_server_ip is not None:
        mqtt_client = mqtt.Client()
        mqtt_client.connect(mqtt_server_ip, 1883, 60)
        mqtt_client.loop_start()
    else:
        raise NameError("No MQTT Server configured")

    pwm = GPIO.PWM(PWM_PIN, 6)  # GPIO 4 for PWM with 6Hz the SSR is zero crossing 
    # Initialization
    duty_cycle = 0  # 0=Relay Open; Power Off | 100=Relay Closed; Full Power

    pwm.start(duty_cycle)
    mqtt_client.publish("heater/command", str(duty_cycle))

    try:
        while True:
            msg = subscribe.simple("coffee/control/Command", hostname=mqtt_server_ip, retained=True)
            duty_cycle = float(msg.payload.decode())
            print("Heater Duty Cycle is now: %s" % duty_cycle)
            pwm.ChangeDutyCycle(duty_cycle)
            mqtt_client.publish("heater/command", str(duty_cycle))
            time.sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        pwm.stop()
        GPIO.cleanup()
