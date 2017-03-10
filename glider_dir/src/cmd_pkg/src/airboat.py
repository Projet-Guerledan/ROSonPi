#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np


def publisher(data):
    pub = rospy.Publisher('state', dstate, queue_size=10)
    rospy.init_node('state', anonymous=True)
    rate = rospy.Rate(10) #10 hz
        msg.dx = data[0]
        msg.dy = data[0]
        msg.dtheta = data[0]
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


def update_pos(msg):
    pos[0] = msg.x
    pos[1] = msg.y
    pos[2] = msg.theta

class Airboat():

    def __init__(self):
        self.power = 0 # 0 to 100%
        self.u = 0
        self.cap = 0
        self.dist2tar = 0

    def update_cmd(msg):
        self.u = msg

    def update_cap(msg):
        self.cap = msg

    def update_tar(msg):
        self.dist2tar = msg

    def command(self):
        rospy.Subscriber('pid/cmd', Float32,update_cmd)
        rospy.Subscriber('navigator/cap',Float32,update_cap)
        rospy.Subscriber('navigator/target',Float32,update_tar)
        #TODO find a control function
        A = 0.5
        B = 1
        self.power = self.power + A*self.dist2tar**2 + B*(self.u-self.cap)
        if diff != 0:
            self.power = min(100,self.power + k*20/diff**2)
        elif k !=0:
            self.power = min(100,self.power-20*np.sqrt(1/k))

        rospy.spin()


def main():
    Airboat()

if __name__ == '__main__':
    main()
