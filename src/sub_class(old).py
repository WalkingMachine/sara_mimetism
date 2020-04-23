#! /usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Pose

class MimetismeWrapper(object):
    def __init__(self):
        rospy.init_node("MimetismeWrapper")
        self.pub = rospy.Publisher('blalaba', JointTrajectory, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        print("Just initialised MimetismeWrapper")

    def main_task(self):
        self.publish(JointTrajectory())
        rospy.spin()

if __name__ == "__main__":
    mime = MimetismeWrapper()
    mime.main_task()
