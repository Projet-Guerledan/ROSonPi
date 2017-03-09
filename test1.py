import RPi.GPIO as gpio
import time

gpio.setmode(gpio.BOARD)
gpio.setup(12,gpio.OUT)

p1 = gpio.PWM(12,50)
p1.start(5.0)
time.sleep(4)
p1.ChangeDutyCycle(5.4)
time.sleep(4)
p1.stop()
