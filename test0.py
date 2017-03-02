import RPi.GPIO as GPIO
import time
import atexit



st='Demarre avec a. Dirige avec zqsd. Stop avec r'

pin1=12
pin2=32

GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin1,GPIO.OUT)
GPIO.setup(pin2,GPIO.OUT)

p1 = GPIO.PWM(pin1,50)
p2 = GPIO.PWM(pin2,50)
p1.start(0.0)
p2.start(0.0)



@atexit.register
def goodbye():
    print "You are now leaving the Python sector."
    stop()


def armMotors():
	global pin1,pin2,p1,p2
        p1=GPIO.PWM(pin1,50)
        p2=GPIO.PWM(pin2,50)
        p1.start(5.0)
        print('Arming Motor 1')
        p2.start(5.0)
        print('Arming Motor 2')
        print('Wait 3s')
        time.sleep(3)

def forward():
	global p1
        p1.ChangeDutyCycle(5.5)
	p2.ChangeDutyCycle(5.5)
        

def turnR():
	global p1,p2
        p1.ChangeDutyCycle(5.0)
        p2.ChangeDutyCycle(5.5)
        

def turnL():
	global p1,p2
        p1.ChangeDutyCycle(5.5)
        p2.ChangeDutyCycle(5.0)
      
def pause():
	global p1,p2
        p1.ChangeDutyCycle(5.0)
        p2.ChangeDutyCycle(5.0)  

def stop():
	global p1,p2
        p1.ChangeDutyCycle(0.0)
        p2.ChangeDutyCycle(0.0)
        p1.stop()
        p2.stop()
        #GPIO.cleanup()

def main():
	global p1,p2
        flag = False
        while not flag:
                entry=raw_input(st)
                if entry in {'a','z','q','s','d','r'}:
                        if entry=='a':
                                armMotors()
                        elif entry == 'z':
                                forward()
                        elif entry == 'q':
                                turnL()
                        elif entry == 's':
                                pause()
                        elif entry == 'd':
                                turnR()
                        elif entry == 'r':
                                stop()
                        else:
                                print('waiting for entry')
                else:
                        
                        print('wrong entry')
                time.sleep(0.02)
                        

if __name__ == "__main__":
	main()
