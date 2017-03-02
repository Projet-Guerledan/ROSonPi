import RPi.GPIO as GPIO
import time


p1=12
p2=32


st='Demarre avec a. Dirige avec zqsd. Stop avec r'

GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin1,GPIO.OUT)
GPIO.setup(pin2,GPIO.OUT)

def armMotors():
        p1=GPIO.PWM(pin1,50)
        p2=GPIO.PWM(pin2,50)
        p1.start(5.0)
        print('Arming Motor 1')
        p2.start(5.0)
        print('Arming Motor 2')
        print('Wait 10s')
        time.sleep(10)

def forward():
        p1.ChangeDutyCycle(6.0)
        time.sleep(5)

def uTurn():
        p2.ChangeDutyCycle()
        time.sleep(6)


def turnR():
        p2.changeDutyCycle()
        time.sleep(2)


def turnL():
        p2.ChangeDutyCycle()
        time.sleep(8)

def stop():
        p1.ChangeDutyCycle(0.0)
        p2.ChangeDutyCycle(0.0)
        time.sleep(2)
        p1.stop()
        p2.stop()
        GPIO.cleanup()

def main():
        flag = False
        while not flag:
                entry=raw_imput(st)
                if entry in {'a','z','q','s','d','r'}:
                        if entry=='a':
                                armMotors()
                        elif entry == 'z':
                                forward()
                        elif entry == 'q':
                                turnL()
                        elif entry == 's':
                                uTurn()
                        elif entry == 'd':
                                turnR()
                        elif entry == 'r':
                                stop()
                        else:
                                print('waiting for entry')
                else:
                        stop()
                        print('ERROR')
                        break

if __name__ == "__main__":
        main()
