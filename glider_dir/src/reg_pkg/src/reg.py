#!/usr/bin/env python

import rospy
from std_msgs.msg
from airboat_msg.msg import cmd
import numpy as np

def update_cmd(data):
    pass

def update_msg(data):
    

def subcriber():
    rospy.init_node()
    #subcriber mission target
    rospy.Subscriber()
    #subcriber position


# publish command
def publisher(data):
    pub = rospy.Publisher('PID', cmd, queue_size=10)
    rospy.init_node('pid_node', anonymous=True)
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()


class PID(object):

    def __init__(self):
        self.kp = 0
        self.kd = 0
        self.ki = 0
        self.u = 0

    def pid_reg(self,x):
        err = x_cmd - x
        err_tot = err_tot+err
        P = kp*err
        I = ki*err_tot*dict
        D = kd * (err - errp)/dt
        u = P+I+D
        return u


def main():
    regulator = PID()
    subcriber()


if __name__ == '__main__':
    main()
