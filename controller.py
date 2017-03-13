#! /usr/bin/python
# Written by Dan Mandle http://dan.mandle.me September 2012
# License: GPL 2.0
# Modified and Adapted by P.Benet and N. MEHDI for Guerledan 2017


import RPi.GPIO as GPIO
import os
from time import *
import time
import serial
import threading
import numpy as np
from numpy import sin,cos,arctan,sqrt
import sys

os.system('clear') #clear the terminal (optional)

st='Demarre avec a. Dirige avec zqsd. Stop avec r\n'

pin1=16
pin2=32
p1=0
p2=0

x=0
y=0
dirx = 0
diry = 0

GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin1,GPIO.OUT)
GPIO.setup(pin2,GPIO.OUT)

p1 = GPIO.PWM(pin1,50)
p2 = GPIO.PWM(pin2,50)
p1.start(0.0)
p2.start(0.0)

def longlat2meter(la,lo):
  rt=6400000 # Earth radius in meter
  x=-(lo*np.pi/180)*rt*cos(la*np.pi/180)
  y=rt*(la*np.pi/180)
  coord = np.array([x,y])
  return coord

def argvect(x,y):
  n = sqrt(x*x+y*y)
  return 2.*arctan(y/(x+n))

class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    self.tps = 0
    self.la = 0
    self.lo = 0
    self.ser = serial.Serial("/dev/ttyUSB1",4800) #starting the stream of info
    self.gps_coord = open('gps_coord.msg','a')
    self.current_value = None
    self.running = True #setting the thread running to true

  def run(self):
    global x,y,dirx,diry
    while gpsp.running:
      line = self.ser.readline() #this will continue to loop and grab EACH set of gpsd info to clear the buffer
      #print(line[0:6])
      if line[0:6] == "$GPGGA" and len(line) > 70:
        self.tps  = float(line[7:17])
        self.la  = float(line[18:20]) + (1./60.)*float(line[20:27])
        self.lo  = float(line[30:33]) + (1./60.)*float(line[33:40])
        vect  = longlat2meter(self.la,self.lo) - longlat2meter(48.19877,3.01385)
        dirx = vect[0] - x
        diry = vect[1] - y
        x = vect[0]
        y = vect[1]
        #data = 'Lat : ' + str(self.la) + ',  Long : '+ str(self.lo) + '  Time: ' + str(self.tps) +'\n'
        data = 'x: ' + str(round(x,2)) + '  y: ' + str(round(y,2)) + '  Time: ' + str(time.ctime()) + '  -  ' + str(time.time()) +'\n'
        print(round(x,2),round(y,2),round(dirx,2),round(diry,2))
        #print data
        self.gps_coord.write(data)


class Razor(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    self.ser = serial.Serial('/dev/ttyAMA0',57600,serial.EIGHTBITS,serial.PARITY_NONE,serial.STOPBITS_ONE,None)
    self.ypr = self.ser.readline()

    self.running = True
  def run(self):
    global dirx,diry
    t0 = time.time()
    while self.running :
      self.ypr= self.ser.readline()
      try:
        self.ypr = self.ypr[5:-2]
        self.ypr = self.ypr.split(',')
        self.ypr = [float(c) for c in self.ypr]
        dirx = cos(self.ypr[0]*np.pi/180)
        diry = -sin(self.ypr[0]*np.pi/180)
      except:
        self.ypr = [666,666,666]

      if time.time() - t0 >=0.99:
        t0 = time.time()
        print "razor : dirx: " + str(round(dirx,2)) + "  diry: " + str(round(diry,2))




def command(c1,c2):
  global p1,p2
  if c1 != -1 :
    p1.ChangeDutyCycle(5.0+c1*0.025)
  else:
    p1.ChangeDutyCycle(0)
  if c2 != -1 :
    p2.ChangeDutyCycle(5.0+c2*0.05)
  else:
    p2.ChangeDutyCycle(0)

class Controller(threading.Thread):

  def __init__(self):
    threading.Thread.__init__(self)
    self.kp = 0.5
    self.kd = 0.2
    self.ki = 0.1

    self.pwrstart = 0.3
    self.ltrans = 20. # length for transitory power

    self.dt = 0.1

    self.dteta = 0
    self.ddteta = 0
    self.i = 0

    self.automatic = False
    self.x1=0
    self.y1=0

    self.power=0

    self.running = True

  def run(self):
    global x,y,dirx,diry
    compteur = 0
    while self.running:

      if self.automatic:
        self.power = self.power + self.pwrstart*self.dt*min(sqrt((x-self.x1)*(x-self.x1) +(y-self.y1)*(y-self.y1))/self.ltrans,1)
        # power is reduced when the target is near
        self.power = min(self.power,sqrt((x-self.x1)*(x-self.x1) +(y-self.y1)*(y-self.y1))/self.ltrans)
        # power is limited
        self.power = min(1,self.power)
        # si la dirction du bateau est significative

        diff = 0
        if dirx*dirx + diry*diry > 0.3 :
          idealdirx = self.x1 - x
          idealdiry = self.y1 - y

          dteta0 = self.dteta
          self.dteta = argvect(diry*idealdiry+dirx*idealdirx, diry*idealdirx-dirx*idealdiry)
          self.ddteta = (self.dteta - dteta0)/self.dt
          self.i = self.i + self.ki*self.dteta*self.dt
          self.i = max(-0.3,min(self.i,0.3))

          # pid on angular command
          diff = self.kp*self.dteta + self.kd*self.ddteta + self.i

          # the differential command is limited
          diff = max(-1,min(diff,1))
          #print(diff, self.i, self.ddteta, self.dteta)
          # power grows when the target is far

        compteur=compteur+1
        if compteur*self.dt >= 1:
          print "  power: " + str(round(self.power,3)) + "  diff: " + str(round(diff,3)) + "dteta: " + str(round(self.dteta,3))
          compteur = 0

        command((10*diff+20)*self.power,(-10*diff+20)*self.power)


        if sqrt((x-self.x1)*(x-self.x1) +(y-self.y1)*(y-self.y1)) < 8 :
          self.automatic = False
          print("target match\n")


      time.sleep(self.dt)

  def settarget(self, x2,y2):
    self.automatic = True
    self.x1 = x2
    self.y1 = y2

    self.power = 0
    self.i = 0

  def turnoff(self):
    self.automatic = False


if __name__ == '__main__':
  gpsp = GpsPoller() # create the thread
  control = Controller()
  boussole = Razor()
  try:
    gpsp.start() # start it up
    control.start()
    boussole.start()
    while True:
      #It may take a second or two to get good data
      #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc
      entry=raw_input(st)
      if entry in {'a','z','q','s','d','r','c','v'}:
        if entry=='a':
          control.turnoff()
          command(0,0) # arm motor
          print('Arming Motor 1 and 2')
          print('Wait 3s')
          time.sleep(3)
        elif entry == 'z':
          control.turnoff()
          command(20,20) #go forward
        elif entry == 'q':
          control.turnoff()
          command(0,20) #turn left
        elif entry == 's':
          control.turnoff()
          command(0,0) #wait
        elif entry == 'd':
          control.turnoff()
          command(20,0) #turn right
        elif entry == 'r':
          control.turnoff()
          command(-1,-1) # unarm motor
        elif entry == 'c':
          print('wait x,y pos\n')
          x2=input("coordonne x")
          y2=input("coordonne y")
          print("target :" + str(x2) +" : " + str(y2))
          control.settarget(x2,y2)
        elif entry == 'v':
          control.turnoff()
          command(40,40)

        else:
          print('waiting for entry')
      else:
        print('wrong entry')
      time.sleep(0.1)

  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "\nKilling Thread..."
    gpsp.running = False
    control.running = False
    boussole.running = False
    gpsp.join() # wait for the thread to finish what it's doing

    boussole.join()
    control.join()
    gpsp.ser.close()
    boussole.ser.close()
  print "Done.\nExiting."
