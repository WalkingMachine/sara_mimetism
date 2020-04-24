#!/usr/bin/env python
# -*- coding: utf-8 -*-
# From https://stackoverflow.com/a/55517239/7243716 or https://stackoverflow.com/questions/35176451/python-code-to-calculate-angle-between-three-point-using-their-3d-coordinates#35178910
import numpy as np
import math
from geometry_msgs.msg import Pose, Point
from moveit_commander import MoveGroupCommander
from tf import transformations

# From https://stackoverflow.com/a/17796482/7243716 or https://stackoverflow.com/questions/17796446/convert-a-list-to-a-string-and-back#17796482
from ast import literal_eval

# From https://stackoverflow.com/a/45028419/7243716 or https://stackoverflow.com/questions/45015268/how-do-i-pipe-output-of-one-python-script-to-another-python-script
import sys

import re


def calculatePose(givenArray):
    """
     # Nombre de points : 17 [0-16]
                        # 0 : nez
                        # 1 : oeil gauche
                        # 2 : oeil droit
                        # 3 : oreille gauche
                        # 4 : oreille droite
                        # 5 : epaule gauche
                        # 6 : epaule droite
                        # 7 : coude gauche
                        # 8 : coude droit
                        # 9 : main gauche
                        # 10 : main droite
                        # 11 : hanche gauche
                        # 12 : hanche droite
                        # 13 : genoux gauche
                        # 14 : genoux droit
                        # 15 : pied gauche
                        # 16 : pied droit
    """
    # Extrait les coordonnees des 3 joints du bras gauche
    # a, b, c = np.array((givenArray[6],givenArray[8],givenArray[10]))
    # Extrait les coordonnees des 3 joints du bras droit
    # epaule gauche
    # 5 : epaule gauche
    a = np.array((givenArray[5]))
    # 7 : coude gauche
    b = np.array((givenArray[7]))
    # 9 : poignet gauche
    c = np.array((givenArray[9]))
    # Vectors
    ab = b - a
    bc = c - b
    # Get position and orientation of right wrist
    x,y,z = c
    dist_x_y = (x**2+y**2)**0.5
    roll = 0
    pitch = math.atan2(y,x)
    yaw = math.atan2(z,dist_x_y)
    orientation = transformations.quaternion_from_euler(roll,pitch,yaw)
    return Pose(c,orientation)



def sendAccordingToPeriodicity(periodicity, cycles_without_sending, last_angles):
    # on incremente le compteur de cycle sans message
    cycles_without_sending += 1
    # si le compteur de cycle correspond à n periode
    if cycles_without_sending % periodicity == 0:
        # on remet le compte à 0 pour ne pas depasser la valeur max du type
        cycles_without_sending = 0

        output_str = reprAngles(last_angles)

        # envoi l'information au prochain dans std out
        sys.stdout.write(output_str)
        # force l'ecriture en .vitant le buffering (pratique pour python2.7)
        sys.stdout.flush()


def reprAngles(last_angles) :
    # calcule les angles avec la moyenne mobile
    angles_to_send = moyenne_mobile(last_angles)
    output_str = str(angles_to_send[0]) + " " + str(angles_to_send[1]) + "\n"
    return output_str


def calculateAngles(givenArray):
    # Extrait les coordonnees des 3 joints du bras gauche
    # a, b, c = np.array((givenArray[6],givenArray[8],givenArray[10]))
    # Extrait les coordonnees des 3 joints du bras droit
    a, b, c = np.array((givenArray[5], givenArray[7], givenArray[9]))
    # Vectors
    ab = b - a
    bc = c - b
    # Get angles on the 2D plane by projecting on it (z is the normal here)
    # TODO: Change when dealing with 3d later
    if np.any(ab != 0):
        angle1 = vg.signed_angle(ab, np.array((1, 0, 0)), vg.basis.neg_z)
    else:
        angle1 = math.nan
    if np.any(bc != 0):
        angle2 = vg.signed_angle(bc, np.array((1, 0, 0)), vg.basis.neg_z)
    else:
        angle2 = math.nan
    return angle1, angle2


"""
pos = joint[-1].pos
orientation = quaternion(vec(joints[-2].pos,joints[-1].pos))
Pose(pos, orientation)
"""

# Garde une liste a jour avec <les max_size> derniers elements
def update_liste_values(liste_values, new_elements, max_size):
    # on verifie si les values ne sont pas nan
    for i, value in enumerate(new_elements):
        # si la value est nan
        if math.isnan(value):
            # si la liste n'est pas vide
            # print("value ", i, "  was nan: ",value)
            if len(liste_values) > 1:
                # on prend sa derniere valeur connue
                new_elements[i] = liste_values[-1][i]
            # sinon on met les valeurs a 0
            else:
                new_elements[i] = 0

    liste_values.append(new_elements)
    while len(liste_values) > max_size:
        liste_values.pop(0)
    return liste_values


def moyenne_mobile(list_values):
    # etabli le diviseur par le nombre de liste de values
    diviseur = len(list_values)
    # etabli le nombre de values dans une liste de value
    dim = len(list_values[0])
    # tableau contenant les moyennes des values
    final_values = []

    # si on a qu'un seul value
    if diviseur < 2:
        return list_values

    # initialise la liste de values finaux à 0
    for i in range(dim):
        final_values.append(0)

    # on calcule la moyenne de chaque value
    for values in list_values:
        for i, value in enumerate(values):
            if value != 0:
                final_values[i] += value / diviseur
            else:
                final_values[i] += 0

    return final_values


def sendCommandToMoveit(PERIODICITY, cycles_without_sending, last_value, latest_state={}):
#    self.group.set_pose_target(userdata.target)
    latest_state.group.set_position_target(Point(last_value))
    plan = latest_state.group.plan()
    latest_state.endState = plan.joint_trajectory.points[len(plan.joint_trajectory.points) - 1].positions
    latest_state.group.execute(plan, wait=False)
    return latest_state


def main():
    # Nombre de cycle necessaire pour envoyer un message (1 msg/4 cycles)
    PERIODICITY = 4
    cycles_without_sending = 0  # variable indiquant le cycle actuel

    # precompilation du regex, indication des motifs de mots à enlever
    regex = re.compile(r"prepro.*")
    last_value = []
    latestState = {'group':MoveGroupCommander("RightArm")}
    while True:
        # Lis ligne dans le pipelin stdin
        line = sys.stdin.readline()
        # enlève les motifs de mot predetermines de la string par substitution
        line = regex.sub("", line)

        # si on a plus d'entree
        if not line:
            # on sort de la boucle while (fin du programme)
            break

        # si on a que des espaces blanc
        if line.isspace() or "[]" in line:
            # on passe à la prochaine iteration
            continue

        # on extirpe seulement la liste les coordonnees de la string
        givenArray = literal_eval(line)[0]['coordinates']

        if len(givenArray) < 3:
            sys.stdout.write(repr(givenArray))
            sys.stdout.flush()
            continue

#        pose = calculatePose(givenArray)
        last_value = np.array((givenArray[9]))
        last_value[1] = -last_value[1]

        # met a jour la liste de toutes les poses calcules depuis la dernière periode
#        last_value = update_liste_values(last_value, pos, PERIODICITY)

#        sendAccordingToPeriodicity(PERIODICITY, cycles_without_sending, last_angles)
        latestState = sendCommandToMoveit(PERIODICITY, cycles_without_sending, last_value, latestState)
""" 
#def __init__(self, move=True, waitForExecution=True, group="RightArm", watchdog=15):
    self.group = MoveGroupCommander(group)
#    curState = self.group.get_current_joint_values()
    self.group.set_pose_target(userdata.target)
#    self.group.set_position_target(xyz)
    plan = self.group.plan()
    self.endState = plan.joint_trajectory.points[len(plan.joint_trajectory.points) - 1].positions
    self.group.execute(plan, wait=False)
#self.group.stop()
#self.group.stop()
"""

main()
