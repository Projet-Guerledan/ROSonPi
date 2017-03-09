#!/usr/bin/env python

import rospy
from std_msgs.msg
from airboat_msg.msg import dstate
import numpy as np

def publisher():
    pub = rospy.Publisher('state', dstate, queue_size=10)
    rospy.init_node('state', anonymous=True)
    rate = rospy.Rate(10) #10 hz
    while not rospy.is_shutdown():
        rospy.loginfo()
        pub.publish(data)
        rate.sleep()

class Airboat():

    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.u = 0

    def state(self):
        #command u
        u1,u2 = self.u
        dx = k1*(u1+u2)*np.cos(self.theta)
        dy = k2*(u1+u2)*np.sin(self.theta)
        dtheta = k3*(u2-u1)
        y = np.array[dx,dy,dz]
        return y


def main():
    overboard = Airboat()
    
