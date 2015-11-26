#!/usr/bin/python

# To be run on Intel Edision board.

from argparse import ArgumentParser
import sys
import socket
import time
import curses
from math import atan2, hypot, degrees

from modules.Nav2Robot import Nav2Robot

import pyupm_joystick12 as upmJoystick12


class NormalizedJoystick:
  #                                  full-[lt,rt],          full-[dn,up]
  def __init__(self, pins=[0,1], xRange = [-0.9, -0.5], yRange = [-0.9, -0.5]):
    self.joy = upmJoystick12.Joystick12(pins[0], pins[1])
    self.xCal = 1/abs(yRange[1] - yRange[0]) # intention XY switch.
    self.yCal = 1/abs(xRange[1] - xRange[0])

    self.xShift = abs(min(xRange))
    self.yShift = abs(min(yRange))

  def x(self):
    return (joy.getYInput() + xShift) * xCal - 0.5

  def y(self):
    return 0.5 - ( (joy.getXInput() + yShift) * yCal )

def getDataFromDevice(device):
	lx = device[0].x()
	ly = device[0].y()
	rx = device[1].x()
	ry = device[1].y()

	jRad = hypot(lx, ly)
	jAng = degrees(atan2(ly, lx))
  rRad = hypot(rx, ry)
	rAng = degrees(atan2(ry, rx))

	if abs(jRad) < 0.1:
		jRad = 0.
		jAng = 0.
	if abs(rRad) < 0.1:
		rAng = 0.
	return jRad, jAng, rAng

def pushScreenText(jRad, jAng, rAng, stdscr):
	stdscr.addstr(2, 2, "   Speed : " + '{:10.4f}'.format(jRad) + " m/s")
	stdscr.addstr(3, 2, " Heading : " + '{:10.4f}'.format(jAng) + " deg")
	stdscr.addstr(4, 2, "Turnrate : " + '{:10.4f}'.format(rAng) + " deg/s")
	stdscr.refresh()

def haltAndDie(stdscr):
	stop_robot()
	sock.close()
	curses.endwin()

def composeAndSendCommand(jRad, jAng, rAng):
	msg=("v " + str(jAng) + " " + str(jRad) + " " + str(rAng) + "\n").encode()
	sock.send( msg )

def mainLoop(stdscr): # Curses main loop.
	stdscr.border(1)
	curses.use_default_colors()
	stdscr.addstr(4, 2, "INITIALIZED AND READY")
	stdscr.noutrefresh()
	curses.doupdate()
	stdscr.clear()

  vme.originate()

	while 1:
		jRad, jAng, rAng = getDataFromDevice()
		pushScreenText(jRad, jAng, rAng, stdscr)
		vme.v(jAng, jRad, rAng)
		time.sleep(0.1)

parser = ArgumentParser(description='Connects a Wii Nunchuck to a VirtualME')
parser.add_argument('-o', '--host', dest='hostname', default='localhost',
    help='Address of turtle server.',
    metavar='HOST')
parser.add_argument('-p', '--port', dest='hostport', type=int, default=5010,
    help='Turtle is listening on this port.',
    metavar='PORT')
parser.add_argument('-s', '--serialdev', dest='serialdev', default='/dev/ttyACM0',
    help='Path to serial device. (On win, this is your COM port.)',
    metavar='/dev/tty???')

args = parser.parse_args()

print("Initializing connection to VirualME...")
vme = Nav2Robot(args.hostname, args.hostport)
print("  done.")

print("Initializing Joy Pad...")
joyPad = []
joyPad.append(NormalizedJoystick(pins=[2,3]))
joyPad.append(NormalizedJoystick())
print("  done.")

# Grab 100 readings from the Arduino to be confident that the
# MCU and serial conection have stabalized.
for i in range(100):
	getDataFromDevice(joyPad)

vme.stop()

# Start the curses loop.
screen = curses.wrapper(mainLoop)
