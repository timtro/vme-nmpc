#!/usr/bin/python
from argparse import ArgumentParser
from modules.Nav2Robot import Nav2Robot
import numpy as np
import time
from datetime import datetime

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

vme.pointat([0, 10])

time.sleep(10)
