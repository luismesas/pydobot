import struct
import threading
import time

import message
import serial
from message import Message


class DobotMagician(threading.Thread):
    on = True
    x = 0.0
    y = 0.0
    z = 0.0
    r = 0.0
    j1 = 0.0
    j2 = 0.0
    j3 = 0.0
    j4 = 0.0

    # joint_angles = [4]

    def __init__(self, port):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.ser = serial.Serial(port,
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)
        is_open = self.ser.isOpen()
        print('%s open' % self.ser.name if is_open else 'failed to open serial port')
        self.start()

    def run(self):
        while self.on:
            msg = self._get_pose()
            if msg is not None:
                if msg.id == message.ID_GET_POSE:
                    self.x = struct.unpack_from('f', msg.params, 0)
                    self.y = struct.unpack_from('f', msg.params, 4)
                    self.z = struct.unpack_from('f', msg.params, 8)
                    self.r = struct.unpack_from('f', msg.params, 12)
                    self.j1 = struct.unpack_from('f', msg.params, 16)
                    self.j2 = struct.unpack_from('f', msg.params, 20)
                    self.j3 = struct.unpack_from('f', msg.params, 24)
                    self.j4 = struct.unpack_from('f', msg.params, 28)
                    pass
                pass
            time.sleep(0.2)

    def close(self):
        self.on = False
        self.lock.acquire()
        self.ser.close()
        print('%s closed' % self.ser.name)
        self.lock.release()

    def _send_command(self, msg):
        self.lock.acquire()
        self._send_message(msg)
        response = self._read_message()
        self.lock.release()
        return response

    def _send_message(self, msg):
        time.sleep(0.1)
        print('>>', msg)
        self.ser.write(msg.bytes())

    def _read_message(self):
        time.sleep(0.1)
        b = self.ser.read_all()
        if len(b) > 0:
            msg = Message(b)
            print('<<', msg)
            return msg
        return

    def _get_pose(self):
        msg = Message()
        msg.id = message.ID_GET_POSE
        return self._send_command(msg)

    def _set_cp_cmd(self, x, y, z):
        msg = Message()
        msg.id = message.ID_SET_CP_CMD
        msg.ctrl = 0x03
        msg.params = bytearray(bytes([0x01]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.append(0x00)
        return self._send_command(msg)

    def move(self, x, y, z):
        self._set_cp_cmd(x, y, z)
