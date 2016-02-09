#!/usr/bin/python
from argparse import ArgumentParser
from modules.Nav2Robot import Nav2Robot
import numpy as np
import time
from datetime import datetime
from math import atan2, degrees, radians

parser = ArgumentParser(description='Sends a virtualME to origin or specified coordinates.')
parser.add_argument('-o',
                    '--host',
                    dest='hostname',
                    default='localhost',
                    help='Address of turtle server.',
                    metavar='HOST')
parser.add_argument('-p',
                    '--port',
                    dest='hostport',
                    type=int,
                    default=5010,
                    help='Turtle is listening on this port.',
                    metavar='PORT')
parser.add_argument('-x', dest='x', type=float, default=0.0, help='Desired x-coordinate.', metavar='X-COORD')
parser.add_argument('-y', dest='y', type=float, default=0.0, help='Desired y-coordinate.', metavar='Y-COORD')
args = parser.parse_args()

# Initialize the TCP/IP socket
print("Initializing connection to VirualME...")
vme = Nav2Robot(args.hostname, args.hostport)
vme.connect()
print("  done.")

# Make sure that, as we start, the robot is stopped.
vme.s()
vme.originate()
vme.o(270)

Ttot = 25  # Total period
subdiv = 24  # Number of sides of the N-gon (N = subdiv).
R = .75  # Radius of the circle intersecting the edges of the N-gon.

th = 360 / subdiv  # Interior angle swept by a side of the N-gon.
phi = 90 - (180 - th) / 2  # Angle between the tangent at a corner and the cord connecting that
#  corner to the adjacent corner.
th = np.radians(th)  # Convert to radians
phi = np.radians(phi)  #    "          "

ival = Ttot / subdiv  # time interval between corners.
speed = subdiv * 2 * R * np.sin(th / 2) / Ttot  # sibdiv * length of N-gon side.
xu = np.cos(phi)  # Get the unit vector along the cord connecting corners.
yu = np.sin(phi)

V = np.array([xu, yu]).reshape(2, 1) * speed  # The velocity to get between the bottom corner and
#  the next corner
# The rotation matrix to switch corners:
Rot = np.array([np.cos(th), -np.sin(th), np.sin(th), np.cos(th)]).reshape(2, 2)

print(V)
print(Rot)

camera = np.array([1.5, 2.5])
chairA = np.array([0, 2 * R])
chairB = chairA + [0, 2 * R]
breakin = chairA - [0, R]
# breakaway = chairA + [0, R]
breakaway = [0, 2.73]
# final_tgt = np.array([-1.33*R, 4*R])
final_tgt = np.array([-1, 4])

print(breakaway, vme.distance_to(breakaway))
print(vme.distance_to([0, 0]))
print(vme.displacement_from([1, 1]), vme.distance_to([1, 1]))
print('Final tgt: ', final_tgt)

vme.avv(speed, degrees(atan2(breakin[1], breakin[0])))  # Move to breakin
time.sleep(np.sqrt(np.sum(breakin[:]**2)) / speed)  #

vme.pointat(camera)
vme.av(float(V[0]), float(V[1]))
while vme.distance_to(breakaway) > .25:
    vme.pointat(camera)
    time.sleep(ival / 2)
    vme.pointat(camera)
    time.sleep(ival / 2)
    V = Rot.dot(V)
    vme.av(float(V[0]), float(V[1]))
    print(vme.displacement_from(breakaway), vme.distance_to(breakaway))
Rot = np.array([np.cos(-th), -np.sin(-th), np.sin(-th), np.cos(-th)]).reshape(2, 2)
while vme.distance_to(final_tgt) > .1:
    vme.pointat(camera)
    time.sleep(ival / 2)
    vme.pointat(camera)
    time.sleep(ival / 2)
    V = Rot.dot(V)
    vme.av(float(V[0]), float(V[1]))
    print(vme.displacement_from(final_tgt), vme.distance_to(final_tgt))
