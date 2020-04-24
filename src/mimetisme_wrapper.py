#! /usr/bin/env python3

import rospy
import sys
import pipes
from subprocess import Popen, PIPE
from shlex import split

class MimetismeWrapper(object):
    def __init__(self, group="RightArm"):
        rospy.init_node("MimetismeWrapper")
        print("Just initialised MimetismeWrapper")

    #Call bash command equivalent "python <openpifpafwebdemo> | python pifpafToMoveit"
    def main_task(self):
        p1 = Popen(split("python3 ../MTI805/openpifpafwebdemo/server.py"), stdout=PIPE)
        p2 = Popen(split("python2 pifpafToMoveit.py"), stdin=p1.stdout)

    def test_task(self):
        p1 = Popen(split("cat ../test_files/pifpaf_feed_3.txt"), stdout=PIPE)
        p2 = Popen(split("python2 pifpafToMoveit.py"), stdin=p1.stdout)


if __name__ == "__main__":
    mime = MimetismeWrapper()
#    mime.main_task()

    mime.test_task()
