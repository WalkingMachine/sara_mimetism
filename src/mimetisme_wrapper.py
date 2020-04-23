#! /usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
import sys
import pipes

class MimetismeWrapper(object):
    def __init__(self, group="RightArm"):
        rospy.init_node("MimetismeWrapper")
#        self.pub = rospy.Publisher('blalaba', JointTrajectory, queue_size=10)
#        self.rate = rospy.Rate(10)  # 10hz
        self.group = MoveGroupCommander(group)
        print("Just initialised MimetismeWrapper")

    def main_task(self):
        while True:
            line = sys.stdin.readline()  # Lis ligne dans le pipelin stdin
#            line = regex.sub("", line)  # enlève les motifs de mot prédeterminés de la string par substitution

            # si on a plus d'entrée
            if not line:
                break  # on sort de la boucle while (fin du programme)

            # si on a que des espaces blanc
            if line.isspace() or "[]" in line:
                continue  # on passe à la prochaine itération

            self.group.set_pose_target(userdata.target)
            plan = self.group.plan()
            self.endState = plan.joint_trajectory.points[len(plan.joint_trajectory.points) - 1].positions
            self.group.execute(plan, wait=False)

""" 
#def __init__(self, move=True, waitForExecution=True, group="RightArm", watchdog=15):
    self.group = MoveGroupCommander(group)
#    curState = self.group.get_current_joint_values()
    self.group.set_pose_target(userdata.target)
#    self.group.set_position_target(xyz)
#    self.group.set_named_target(userdata.target)
    plan = self.group.plan()
    self.endState = plan.joint_trajectory.points[len(plan.joint_trajectory.points) - 1].positions
    self.group.execute(plan, wait=False)
#self.group.stop()
#self.group.stop()
"""


if __name__ == "__main__":
    mime = MimetismeWrapper()
    mime.main_task()
