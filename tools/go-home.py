#!/usr/bin/python
from argparse import ArgumentParser
import socket
import numpy as np


def stop_robot():
    '''
	Stop the robot by sending it 's'.
	'''
    return sock.send("s\n".encode())


def locate_robot():
    sock.send('q\n'.encode())
    return [float(str) for str in sock.makefile().readline().split()]


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
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((args.hostname, args.hostport))
print("  done.")

# Make sure that, as we start, the robot is stopped.
stop_robot()

X = locate_robot()
r, phi = str(np.sqrt(X[0]**2 + X[1]**2)), np.degrees(np.arctan2(X[1], X[0]))

msg = 'MV ' + str(r) + ' ' + str(phi - X[2] + 180) + '\n'
sock.send(msg.encode())

print(msg)

Xprev = np.array(X)
while True:
    Xnew = np.array(locate_robot())
    print(np.absolute(np.sqrt(np.sum((Xprev[:1] - np.array([0, 0])))**2)))
    if np.absolute(np.sqrt(np.sum((Xprev[:1] - np.array([0, 0])))**2)) < .03:
        break
    Xprev = Xnew

sock.send('o 0\n'.encode())

while True:
    Xnew = np.array(locate_robot())
    if np.absolute(Xprev[2]) < 1:
        break
    Xprev = Xnew

print('done')
stop_robot()
