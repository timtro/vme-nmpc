#!/usr/bin/python

# To be run on Intel Edision board.

from argparse import ArgumentParser
import sys, signal
import socket
import time
import curses
from math import atan2, hypot, degrees

from modules.Nav2Robot import Nav2Robot
from modules.JoyPad import JoyPad

def getDataFromDevice(device):
  lx = device.lx()
  ly = -device.ly()
  rx = device.rx()
  ry = device.ry()


  if device.buttonStates['base6']:
    vme.originate()

  speed = hypot(lx, ly)
  heading = degrees(atan2(ly, lx))-90
  turnRate = rx * -180

  if speed < 0.05:
    speed = 0.
    heading = 0.
  if abs(rx) < 0.01:
    turnRate = 0.
  return speed, heading, turnRate

def pushScreenText(speed, heading, turnRate, stdscr):
  stdscr.addstr(2, 2, "   Speed : " + '{:10.4f}'.format(speed) + " m/s")
  stdscr.addstr(3, 2, " Heading : " + '{:10.4f}'.format(heading) + " deg")
  stdscr.addstr(4, 2, "Turnrate : " + '{:10.4f}'.format(turnRate) + " deg/s")
  stdscr.refresh()

def haltAndDie():
  vme.stop()
  curses.endwin()
  joyPad.cancel()

def signal_handler(signal, frame):
  haltAndDie()
  sys.exit(0)

def composeAndSendCommand(speed, heading, turnRate):
  vme.v(heading, speed, turnRate)

def mainLoop(stdscr): # Curses main loop.
  stdscr.border(1)
  curses.use_default_colors()
  stdscr.addstr(4, 2, "INITIALIZED AND READY")
  stdscr.noutrefresh()
  curses.doupdate()
  stdscr.clear()

  vme.originate()

  while 1:
    speed, heading, turnRate = getDataFromDevice(joyPad)
    pushScreenText(speed, heading, turnRate, stdscr)
    vme.v(heading, speed, turnRate)
    time.sleep(0.1)

parser = ArgumentParser(description='Connects a Wii Nunchuck to a VirtualME')
parser.add_argument('-o', '--host', dest='hostname', default='localhost',
    help='Address of turtle server.',
    metavar='HOST')
parser.add_argument('-p', '--port', dest='hostport', type=int, default=5010,
    help='Turtle is listening on this port.',
    metavar='PORT')

args = parser.parse_args()

signal.signal(signal.SIGINT, signal_handler)

print("Initializing connection to VirualME...")
vme = Nav2Robot(args.hostname, args.hostport)
vme.connect()
print("  done.")

print("Initializing Joy Pad...")
joyPad = JoyPad()
joyPad.start()
print("  done.")

vme.stop()

# Start the curses loop.
screen = curses.wrapper(mainLoop)