# From https://stackoverflow.com/a/55517239/7243716 or https://stackoverflow.com/questions/35176451/python-code-to-calculate-angle-between-three-point-using-their-3d-coordinates#35178910
import numpy as np
import vg
import math
from geometry_msgs.msg import Pose

# From https://stackoverflow.com/a/17796482/7243716 or https://stackoverflow.com/questions/17796446/convert-a-list-to-a-string-and-back#17796482
from ast import literal_eval

# From https://stackoverflow.com/a/45028419/7243716 or https://stackoverflow.com/questions/45015268/how-do-i-pipe-output-of-one-python-script-to-another-python-script
import sys

import re


def main():
    # Nombre de cycle nécessaire pour envoyer un message (1 msg/4 cycles)
    PERIODICITY = 4
    cycles_without_sending = 0  # variable indiquant le cycle actuel

    # précompilation du regex, indication des motifs de mots à enlever
    regex = re.compile(r"prepro.*")
    last_angles = []
    while True:
        # Lis ligne dans le pipelin stdin
        line = sys.stdin.readline()
        # enlève les motifs de mot prédeterminés de la string par substitution
        line = regex.sub("", line)

        # si on a plus d'entrée
        if not line:
            # on sort de la boucle while (fin du programme)
            break

        # si on a que des espaces blanc
        if line.isspace() or "[]" in line:
            # on passe à la prochaine itération
            continue

        # on extirpe seulement la liste les coordonnées de la string
        givenArray = literal_eval(line)[0]['coordinates']

        if len(givenArray) < 3:
            sys.stdout.write(repr(givenArray))
            sys.stdout.flush()
            continue

        angle1, angle2 = calculateAngles(givenArray)

        # met a jour la liste de tous les angles calculés depuis la dernière période
        last_angles = update_liste_angles(last_angles, [angle1, angle2], PERIODICITY)

        sendAccordingToPeriodicity(PERIODICITY, cycles_without_sending, last_angles)


def sendAccordingToPeriodicity(periodicity, cycles_without_sending, last_angles):
    # on incrémente le compteur de cycle sans message
    cycles_without_sending += 1
    # si le compteur de cycle correspond à n période
    if cycles_without_sending % periodicity == 0:
        # on remet le compte à 0 pour ne pas dépasser la valeur max du type
        cycles_without_sending = 0

        output_str = reprAngles(last_angles)

        # envoi l'information au prochain dans std out
        sys.stdout.write(output_str)
        # force l'écriture en .vitant le buffering (pratique pour python2.7)
        sys.stdout.flush()


def reprAngles(last_angles) -> str:
    # calcule les angles avec la moyenne mobile
    angles_to_send = moyenne_mobile(last_angles)
    output_str = str(angles_to_send[0]) + " " + str(angles_to_send[1]) + "\n"
    return output_str


def calculateAngles(givenArray):
    # Extrait les coordonnées des 3 joints du bras gauche
    # a, b, c = np.array((givenArray[6],givenArray[8],givenArray[10]))
    # Extrait les coordonnées des 3 joints du bras droit
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
def update_liste_angles(liste_angles, new_elements, max_size):
    # on verifie si les angle ne sont pas nan
    for i, angle in enumerate(new_elements):
        # si l<angle est nan
        if math.isnan(angle):
            # si la liste n'est pas vide
            # print("angle ", i, "  was nan: ",angle)
            if len(liste_angles) > 1:
                # on prend sa derniere valeur connue
                new_elements[i] = liste_angles[-1][i]
            # sinon on met les valeurs a 0
            else:
                new_elements[i] = 0

    liste_angles.append(new_elements)
    while len(liste_angles) > max_size:
        liste_angles.pop(0)
    return liste_angles


def moyenne_mobile(list_values):
    # etabli le diviseur par le nombre de liste de values
    diviseur = len(list_values)
    # établi le nombre de values dans une liste de value
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


main()
