#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np



# publish command
def publisher(data):
    pub = rospy.Publisher('pid/cmd',Float32, queue_size=10)
    rospy.init_node('pid_node', anonymous=True)
    rate = rospy.Rate(10) #10hz
    msg = data[1]
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


class PID(object):

    def __init__(self):
        self.kp = 0
        self.kd = 0
        self.ki = 0
        self.u = 0
        self.err = 0
        self.err_hist = []
        self.errp = 0

    def pid_cap(self):
        self.err_hist.append(self.err)
        if len(self.err_hist) > 0:
            errp = err_hist[len(self.err_hist)]
            err_tot = sum(self.err_hist)
        else:
            errp = 0
            err_tot = 0
        dt = 0.05
        # PID
        P = self.kp*np.arctan(np.tan(self.err/2))
        I = self.ki*err_tot*dt
        D = self.kd*np.sin(err - errp)/dt
        u = P+I+D
        publisher(u)


def main():
    PID()


if __name__ == '__main__':
    main()
