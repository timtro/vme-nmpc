#!/usr/bin/python3

from gi.repository import Gtk, Gdk, GObject

from argparse import ArgumentParser, RawTextHelpFormatter
import sys, signal
import socket
import time
import curses
from math import atan2, hypot, degrees

from modules.Nav2Robot import Nav2Robot
from modules.JoyPad import JoyPad, NullJoyPad


def mainLoop(stdscr):  # Curses main loop.
    stdscr.border(1)
    curses.use_default_colors()
    stdscr.addstr(4, 2, "INITIALIZED AND READY")
    stdscr.noutrefresh()
    curses.doupdate()
    stdscr.clear()
    stdscr.nodelay(True)
    curses.noecho()
    curses.cbreak()

    vme.originate()

    key = ''
    while key != curses.KEY_BACKSPACE:

        key = stdscr.getch()
        speed, heading, turnRate = keyBoardHandler(key)
        if speed == None:
            speed, heading, turnRate = getDataFromDevice(joyPad)

        pushScreenText(speed, heading, turnRate, stdscr)

        if speed == 0 and turnRate == 0:
            vme.stop()
        else:
            vme.v(heading, speed, turnRate)

        speed, heading, turnRate = 0, 0, 0
        curses.flushinp()
        time.sleep(0.15)


def keyBoardHandler(key=''):
    if key == ord(' '):
        return 0, 0, 0
    if key == '':
        return None, None, None
    elif key == curses.KEY_UP:
        speed, heading, turnRate = 0.3, 0, 0
    elif key == curses.KEY_DOWN:
        speed, heading, turnRate = -0.3, 0, 0
    elif key == ord('w'):
        speed, heading, turnRate = 0.3, 0, 0
    elif key == ord('s'):
        speed, heading, turnRate = -0.3, 0, 0
    elif key == ord('a'):
        speed, heading, turnRate = 0.3, 90, 0
    elif key == ord('d'):
        speed, heading, turnRate = 0.3, -90, 0
    elif key == ord('e'):
        speed, heading, turnRate = 0, 0, -30
    elif key == ord('q'):
        speed, heading, turnRate = 0, 0, 30
    else:
        return None, None, None

    return speed, heading, turnRate


def pushScreenText(speed, heading, turnRate, stdscr):
    stdscr.addstr(2, 2, "   Speed : " + '{:10.4f}'.format(speed) + " m/s")
    stdscr.addstr(3, 2, " Heading : " + '{:10.4f}'.format(heading) + " deg")
    stdscr.addstr(4, 2, "Turnrate : " + '{:10.4f}'.format(turnRate) + " deg/s")
    stdscr.refresh()


def composeAndSendCommand(speed, heading, turnRate):
    vme.v(heading, speed, turnRate)


def getDataFromDevice(device):
    lx = device.lx() * .5
    ly = -device.ly() * .5
    rx = device.rx() * .5

    # Push R-thumb to assign origin to currebt position.
    # if device.buttonStates['base']:
    # vme.originate()

    speed = hypot(lx, ly)
    heading = degrees(atan2(ly, lx)) - 90
    turnRate = rx * -180

    if speed < 0.05:
        speed = 0.
        heading = 0.
    if abs(rx) < 0.01:
        turnRate = 0.
    return speed, heading, turnRate


def haltAndDie():
    vme.stop()
    curses.endwin()
    joyPad.cancel()


def signalHandler(signal, frame):
    haltAndDie()
    sys.exit(0)


class MainWindow(Gtk.Window):

    def __init__(self):
        Gtk.Window.__init__(self, title='Control virtualME')
        self.settings = Gtk.Settings.get_default()
        self.settings.set_property("gtk-application-prefer-dark-theme", True)
        self.connect("delete-event", Gtk.main_quit)
        self.show_all()


parser = ArgumentParser(description="""Use the keyboard or a joypad/joystick to control virtualME.
    Keyboard controls:
    <BACKSPACE>         Exit
    <SPACEBAR>          STOP
    a, d                Move left/right
    w, s                Move forward/backward
    q, e                Rotate left, right""",
                        formatter_class=RawTextHelpFormatter)
parser.add_argument('-o',
                    '--host',
                    dest='hostname',
                    default='localhost',
                    help='Address of turtle server. (Default: localhost)',
                    metavar='HOST')
parser.add_argument('-p',
                    '--port',
                    dest='hostport',
                    type=int,
                    default=5010,
                    help='Turtle is listening on this port. (Default: 5010)',
                    metavar='PORT')
parser.add_argument('-j',
                    '--joydev',
                    dest='joyStickDevicePath',
                    default='/dev/input/js0',
                    help='Path to joystick device. (Default: /dev/input/js0)',
                    metavar='DEV')
parser.add_argument('--nojoy',
                    action='store_true',
                    help='Do not look for joypad/stick. Just use the keyboard.\n(Default is to use joypad/stick.)')

# Start the curses loop.
#screen = curses.wrapper(mainLoop)

if __name__ == "__main__":
    args = parser.parse_args()

    # Handle Ctrl+C gracefully.
    signal.signal(signal.SIGINT, signalHandler)

    print("Initializing connection to VirualME...")
    vme = Nav2Robot(args.hostname, args.hostport)
    vme.connect()
    print("  done.")

    if args.nojoy:
        joyPad = NullJoyPad()
    else:
        print("Initializing Joy Pad...")
        joyPad = JoyPad(args.joyStickDevicePath)
        joyPad.start()
        print("  done.")

    vme.stop()

    window = MainWindow()
    Gtk.main()
