#!/usr/bin/python3

import sys
import signal
import time
from argparse import ArgumentParser, RawTextHelpFormatter
from math import atan2, hypot, degrees

from modules.Nav2Robot import Nav2Robot
from modules.JoyPad import JoyPad, NullJoyPad


def main_loop(vme, pad):
    vme.originate()
    while 1:
        speed, heading, turnRate = stick_data_from_device(pad)
        speed *= boost_status(pad)

        if speed == 0 and turnRate == 0:
            vme.stop()
        else:
            vme.v(heading, speed, turnRate)

        speed, heading, turnRate = 0, 0, 0
        time.sleep(0.15)


def stick_data_from_device(device):
    lx = device.lx() * .5
    ly = -device.ly() * .5
    rx = device.rx() * .5

    speed = hypot(lx, ly)
    heading = degrees(atan2(ly, lx)) - 90
    turnRate = rx * -180

    if speed < 0.05:
        speed = 0.
        heading = 0.
    if abs(rx) < 0.01:
        turnRate = 0.
    return speed, heading, turnRate


def boost_status(device):
    if device.buttonStates['trigger'] == 1:
        return 1.5
    else:
        return 1


parser = ArgumentParser(
    description="""Use the keyboard or a joypad/joystick to control virtualME.
    Keyboard controls:
    <BACKSPACE>         Exit
    <SPACEBAR>          STOP
    a, d                Move left/right
    w, s                Move forward/backward
    q, e                Rotate left, right""",
    formatter_class=RawTextHelpFormatter)
parser.add_argument(
    '-o',
    '--host',
    dest='hostname',
    default='localhost',
    help='Address of turtle server. (Default: localhost)',
    metavar='HOST')
parser.add_argument(
    '-p',
    '--port',
    dest='hostport',
    type=int,
    default=5010,
    help='Turtle is listening on this port. (Default: 5010)',
    metavar='PORT')
parser.add_argument(
    '-j',
    '--joydev',
    dest='joyStickDevicePath',
    default='/dev/input/js0',
    help='Path to joystick device. (Default: /dev/input/js0)',
    metavar='DEV')
parser.add_argument(
    '--nojoy', action='store_true', help='Do not look for joypad/stick.')

args = parser.parse_args()

print("Initializing connection to VirualME...")
virtualME = Nav2Robot(args.hostname, args.hostport)
virtualME.connect()
print("  done.")

if args.nojoy:
    joyPad = NullJoyPad()
else:
    print("Initializing Joy Pad...")
    joyPad = JoyPad(args.joyStickDevicePath)
    joyPad.start()
    print("  done.")


def signalHandler(signal, frame):
    virtualME.stop()
    joyPad.cancel()
    sys.exit(0)


# Handle Ctrl+C gracefully.
signal.signal(signal.SIGINT, signalHandler)

print("\nThere's no more curses-dependency, so there's no more output.")
print("If something goes wrong, you\'re debugging. Good luck.\n")
print("\"Hold on to your butts.\"")
print("        --John Raymond Arnold")
print("          Jurassic Park\n")

virtualME.stop()
main_loop(virtualME, joyPad)
