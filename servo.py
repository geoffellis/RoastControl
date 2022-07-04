import RPi.GPIO as GPIO
import time

servoPIN = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 4 for PWM with 50Hz
p.start(2.5) # Initialization
#Typically 1.5 ms is center and min is 1ms and max is 2ms
# @50hz ( 20 ms )
#By experiment found min is ~ 3 max is 11 middle should be ~6
dutyCycle = 3
try:
  while True:
    dutyCycle = dutyCycle + 0.05
    if dutyCycle>11 :
        dutyCycle=3
    p.ChangeDutyCycle(dutyCycle)
    time.sleep(0.5)
except KeyboardInterrupt:
  p.stop()
  GPIO.cleanup()