# Based on work by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import struct
import array
import time
from fcntl import ioctl
from threading import Thread


class JoyPad(Thread):
    # These constants were borrowed from linux/input.h
    axisNames = {
        0x00: 'x',
        0x01: 'y',
        0x02: 'z',
        0x03: 'rx',
        0x04: 'ry',
        0x05: 'rz',
        0x06: 'trottle',
        0x07: 'rudder',
        0x08: 'wheel',
        0x09: 'gas',
        0x0a: 'brake',
        0x10: 'hat0x',
        0x11: 'hat0y',
        0x12: 'hat1x',
        0x13: 'hat1y',
        0x14: 'hat2x',
        0x15: 'hat2y',
        0x16: 'hat3x',
        0x17: 'hat3y',
        0x18: 'pressure',
        0x19: 'distance',
        0x1a: 'tilt_x',
        0x1b: 'tilt_y',
        0x1c: 'tool_width',
        0x20: 'volume',
        0x28: 'misc',
    }

    buttonNames = {
        0x120: 'trigger',
        0x121: 'thumb',
        0x122: 'thumb2',
        0x123: 'top',
        0x124: 'top2',
        0x125: 'pinkie',
        0x126: 'base',
        0x127: 'base2',
        0x128: 'base3',
        0x129: 'base4',
        0x12a: 'base5',
        0x12b: 'base6',
        0x12f: 'dead',
        0x130: 'a',
        0x131: 'b',
        0x132: 'c',
        0x133: 'x',
        0x134: 'y',
        0x135: 'z',
        0x136: 'tl',
        0x137: 'tr',
        0x138: 'tl2',
        0x139: 'tr2',
        0x13a: 'select',
        0x13b: 'start',
        0x13c: 'mode',
        0x13d: 'thumbl',
        0x13e: 'thumbr',
        0x220: 'dpad_up',
        0x221: 'dpad_down',
        0x222: 'dpad_left',
        0x223: 'dpad_right',

        # XBox 360 controller uses these codes.
        0x2c0: 'dpad_left',
        0x2c1: 'dpad_right',
        0x2c2: 'dpad_up',
        0x2c3: 'dpad_down',
    }

    def __init__(self, joyStickDevicePath='/dev/input/js0'):
        super(JoyPad, self).__init__()
        self.daemon = True
        self.cancelled = False
        self.joyStickName = ''

        self.axisStates = {}
        self.buttonStates = {}
        self.axisMap = []
        self.buttonMap = []

        self.joyStickFd = open(joyStickDevicePath, 'rb')

        # Get the device name.
        buf = array.array('u', ['\0'] * 64)
        ioctl(self.joyStickFd, 0x80006a13 + (0x10000 * len(buf)),
              buf)  # JSIOCGNAME(len)
        self.joyStickName = buf.tostring()

        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(self.joyStickFd, 0x80016a11, buf)  # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.joyStickFd, 0x80016a12, buf)  # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.joyStickFd, 0x80406a32, buf)  # JSIOCGAXMAP

        for axis in buf[:num_axes]:
            axis_name = JoyPad.axisNames.get(axis, 'unknown(0x%02x)' % axis)
            self.axisMap.append(axis_name)
            self.axisStates[axis_name] = 0.0

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.joyStickFd, 0x80406a34, buf)  # JSIOCGBTNMAP

        for btn in buf[:num_buttons]:
            btn_name = JoyPad.buttonNames.get(btn, 'unknown(0x%03x)' % btn)
            self.buttonMap.append(btn_name)
            self.buttonStates[btn_name] = 0

    def update(self):
        evbuf = self.joyStickFd.read(8)
        if evbuf:
            time, value, type, number = struct.unpack('IhBB', evbuf)
            if type & 0x01:
                button = self.buttonMap[number]
                if button:
                    self.buttonStates[button] = value
            if type & 0x02:
                axis = self.axisMap[number]
                if axis:
                    fvalue = value / 32767.0
                    self.axisStates[axis] = fvalue

    def cancel(self):
        self.cancelled = True

    def run(self):
        while not self.cancelled:
            self.update()
            time.sleep(0.001)

    def __exit__(self):
        self.joyStickFd.close()

    def lx(self):
        return self.axisStates['x']

    def ly(self):
        return self.axisStates['y']

    def rx(self):
        return self.axisStates['z']  # for my older model
        # return self.axisStates['rx']

    def ry(self):
        return self.axisStates['rz']  # for my older model
        # return self.axisStates['ry']


class NullJoyPad:

    axisStates = {}
    buttonStates = {}
    axisMap = []
    buttonMap = []

    for key, val in JoyPad.axisNames.items():
        axisStates[val] = 0

    for key, val in JoyPad.buttonNames.items():
        buttonStates[val] = 0

    def lx(self):
        return 0

    def ly(self):
        return 0

    def rx(self):
        return 0

    def ry(self):
        return 0
