#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np

class Navigator(object):

    def __init__(self,mission=[]):
        self.mx = 0
        self.my = 0
        self.cap = 0
        self.mission = mission
        self.target = mission[0]
        self.start = [0,0]

    def publisher(data):
        pub1 = rospy.Publisher('navigator/err',Float32, queue_size=1)
        pub2 = rospy.Publisher('navigator/cap',Float32,queue_size=1)
        pub3 = rospy.Publisher('navigator/target',Float32,queue_size=1)
        rospy.init_node('navigation', anonymous=True)
        rate = rospy.Rate(10)
        msg1 = data[0]
        msg2 = data[1]
        msg3 = data[2]
        while not rospy.is_shutdown():
            pub1.publish(msg1)
            pub2.publish(msg2)
            pub3.publish(msg3)
            rate.sleep()

    def hook():
        print("Shutdown !!!")

    def control(self):
        phi = np.arctan2(self.target[1]-self.start[1], self.target[0]-self.start[0])
        v=np.array([[self.target[0]-self.start[0],
        self.target[1]-self.start[1]],
        [self.mx-self.start[0],self.my-self.start[1]]])
        #distance to the line
        dist = np.linalg.det(v)/np.linalg.norm(np.array([self.target[0]-self.start[0],
        self.target[1]-self.start[1]])
        # cap to follow
        thetacap = phi - np.arctan(dist)
        # distance to the target
        dist_targ = np.linalg.norm(np.array([[self.target[0]-self.mx],[self.target[1]-self.my]]))
        err = thetacap - self.cap
        data = [err,self.cap,dist_targ]
        publisher(data)
        # next target if
        b = np.array(self.target[1]-self.start[1], self.target[0]-self.start[0])
        m = np.array()[elf.target[0]-self.start[0],self.target[1]-self.start[1]])
        bt = np.linalg.transpose(b)
        flag = np.dot(bt,m)
        if flag <= 0 and len(mission)>0:
            self.start = self.target
            mission.pop(0)
            self.target = mission[0]
        elif flag<=0 and len(mission) ==0:
            rospy.on_shutdown(hook())

def main():
    foo = open(path, mode='r')
    for line in foo:
        # TODO: add mission planer
    Navigator(mission)

if __name__ == '__main__':
    main()
