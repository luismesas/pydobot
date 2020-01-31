import struct
import threading
import time

import serial
from serial.tools import list_ports

from pydobot.message import Message

MODE_PTP_JUMP_XYZ = 0x00
MODE_PTP_MOVJ_XYZ = 0x01
MODE_PTP_MOVL_XYZ = 0x02
MODE_PTP_JUMP_ANGLE = 0x03
MODE_PTP_MOVJ_ANGLE = 0x04
MODE_PTP_MOVL_ANGLE = 0x05
MODE_PTP_MOVJ_INC = 0x06
MODE_PTP_MOVL_INC = 0x07
MODE_PTP_MOVJ_XYZ_INC = 0x08
MODE_PTP_JUMP_MOVL_XYZ = 0x09

STEP_PER_CRICLE = 360.0 / 1.8 * 10.0 * 16.0
MM_PER_CRICLE = 3.1415926535898 * 36.0


class DobotException(Exception):
    pass


class Dobot:

    def __init__(self, port=None, verbose=False):

        self.verbose = verbose
        self.lock = threading.Lock()
        if port is None:
            # Find the serial port
            ports = list_ports.comports()
            for thing in ports:
                if thing.vid in (4292, 6790):
                    if self.verbose:
                        print("Found a com port to talk to DOBOT.")
                        print(thing)
                    port = thing.device
                    break
            else:
                raise DobotException("Device not found!")

        self.ser = serial.Serial(port,
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)

        if self.verbose:
            is_open = self.ser.isOpen()
            print('pydobot: %s open' % self.ser.name if is_open else 'failed to open serial port')

        self._set_queued_cmd_start_exec()
        self._set_queued_cmd_clear()
        self._set_ptp_joint_params(200, 200, 200, 200, 200, 200, 200, 200)
        self._set_ptp_coordinate_params(velocity=200, acceleration=200)
        self._set_ptp_jump_params(10, 200)
        self._set_ptp_common_params(velocity=100, acceleration=100)
        self._get_pose()

    def close(self):
        with self.lock:
            self.ser.close()
            if self.verbose:
                print('pydobot: %s closed' % self.ser.name)

    def _send_command(self, msg):
        with self.lock:
            self.ser.reset_input_buffer()
            self._send_message(msg)
            return self._read_message()

    def _send_message(self, msg):
        time.sleep(0.1)
        if self.verbose:
            print('pydobot: >>', msg)
        self.ser.write(msg.bytes())

    def _read_message(self):

        # Search for begin
        begin_found = False
        last_byte = None
        tries = 5
        while not begin_found and tries > 0:
            current_byte = ord(self.ser.read(1))
            if current_byte == 170:
                if last_byte == 170:
                    begin_found = True
            last_byte = current_byte
            tries = tries - 1
        if begin_found:
            payload_length = ord(self.ser.read(1))
            payload_checksum = self.ser.read(payload_length + 1)
            if len(payload_checksum) == payload_length + 1:
                b = bytearray([0xAA, 0xAA])
                b.extend(bytearray([payload_length]))
                b.extend(payload_checksum)
                msg = Message(b)
                if self.verbose:
                    print('Lenght', payload_length)
                    print(payload_checksum)
                    print('MessageID:', payload_checksum[0])
                    print('pydobot: <<', ":".join('{:02x}'.format(x) for x in b))
                return msg
        return

    def _get_pose(self):
        msg = Message()
        msg.id = 10
        response = self._send_command(msg)
        self.x = struct.unpack_from('f', response.params, 0)[0]
        self.y = struct.unpack_from('f', response.params, 4)[0]
        self.z = struct.unpack_from('f', response.params, 8)[0]
        self.r = struct.unpack_from('f', response.params, 12)[0]
        self.j1 = struct.unpack_from('f', response.params, 16)[0]
        self.j2 = struct.unpack_from('f', response.params, 20)[0]
        self.j3 = struct.unpack_from('f', response.params, 24)[0]
        self.j4 = struct.unpack_from('f', response.params, 28)[0]
        if self.verbose:
            print("pydobot: x:%03.1f y:%03.1f z:%03.1f r:%03.1f j1:%03.1f j2:%03.1f j3:%03.1f j4:%03.1f" %
                  (self.x, self.y, self.z, self.r, self.j1, self.j2, self.j3, self.j4))
        return response

    def _set_cp_cmd(self, x, y, z):
        msg = Message()
        msg.id = 91
        msg.ctrl = 0x03
        msg.params = bytearray(bytes([0x01]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.append(0x00)
        return self._send_command(msg)

    def _set_ptp_joint_params(self, v_x, v_y, v_z, v_r, a_x, a_y, a_z, a_r):
        msg = Message()
        msg.id = 80
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', v_x)))
        msg.params.extend(bytearray(struct.pack('f', v_y)))
        msg.params.extend(bytearray(struct.pack('f', v_z)))
        msg.params.extend(bytearray(struct.pack('f', v_r)))
        msg.params.extend(bytearray(struct.pack('f', a_x)))
        msg.params.extend(bytearray(struct.pack('f', a_y)))
        msg.params.extend(bytearray(struct.pack('f', a_z)))
        msg.params.extend(bytearray(struct.pack('f', a_r)))
        return self._send_command(msg)

    def _set_ptp_coordinate_params(self, velocity, acceleration):
        msg = Message()
        msg.id = 81
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    def _set_ptp_jump_params(self, jump, limit):
        msg = Message()
        msg.id = 82
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', jump)))
        msg.params.extend(bytearray(struct.pack('f', limit)))
        return self._send_command(msg)

    def _set_ptp_common_params(self, velocity, acceleration):
        msg = Message()
        msg.id = 83
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    def _set_ptp_cmd(self, x, y, z, r, mode):
        msg = Message()
        msg.id = 84
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray([mode]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        return self._send_command(msg)

    def _set_end_effector_suction_cup(self, enable=False):
        msg = Message()
        msg.id = 62
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)

    def _set_end_effector_gripper(self, enable=False):
        msg = Message()
        msg.id = 63
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)

    def _set_end_effector_laser(self, power=255, enable=False):
        """Enables the laser. Power from 0 to 255. """
        msg = Message()
        msg.id = 61
        msg.ctrl = 0x03
        msg.params = bytearray([])
        # msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        # Assuming the last byte is power. Seems to have little effect
        msg.params.extend(bytearray([power]))
        return self._send_command(msg)

    def _set_queued_cmd_start_exec(self):
        msg = Message()
        msg.id = 240
        msg.ctrl = 0x01
        return self._send_command(msg)

    def _set_queued_cmd_stop_exec(self):
        msg = Message()
        msg.id = 241
        msg.ctrl = 0x01
        return self._send_command(msg)

    def _set_queued_cmd_clear(self):
        msg = Message()
        msg.id = 245
        msg.ctrl = 0x01
        return self._send_command(msg)

    def _get_queued_cmd_current_index(self):
        msg = Message()
        msg.id = 246
        response = self._send_command(msg)
        if response and response.id == 246:
            return self._extract_cmd_index(response)
        else:
            return -1

    @staticmethod
    def _extract_cmd_index(response):
        return struct.unpack_from('I', response.params, 0)[0]

    def wait_for_cmd(self, cmd_id):
        current_cmd_id = self._get_queued_cmd_current_index()
        while cmd_id > current_cmd_id:
            if self.verbose:
                print("Current-ID", current_cmd_id)
                print("Waiting for", cmd_id)
            time.sleep(0.5)
            current_cmd_id = self._get_queued_cmd_current_index()

    def _set_home_cmd(self):
        msg = Message()
        msg.id = 31
        msg.ctrl = 0x03
        msg.params = bytearray([])
        return self._send_command(msg)

    def _set_arc_cmd(self, x, y, z, r, cir_x, cir_y, cir_z, cir_r):
        msg = Message()
        msg.id = 101
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', cir_x)))
        msg.params.extend(bytearray(struct.pack('f', cir_y)))
        msg.params.extend(bytearray(struct.pack('f', cir_z)))
        msg.params.extend(bytearray(struct.pack('f', cir_r)))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        return self._send_command(msg)

    def _set_home_coordinate(self, x, y, z, r):
        msg = Message()
        msg.id = 30
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        return self._send_command(msg)

    def move_to(self, x, y, z, r=0., mode=MODE_PTP_MOVJ_XYZ):
        return self._extract_cmd_index(self._set_ptp_cmd(x, y, z, r, mode))

    def go_arc(self, x, y, z, r, cir_x, cir_y, cir_z, cir_r):
        return self._extract_cmd_index(self._set_arc_cmd(x, y, z, r, cir_x, cir_y, cir_z, cir_r))

    def suck(self, enable):
        return self._extract_cmd_index(self._set_end_effector_suction_cup(enable))

    def set_home(self, x, y, z, r=0.):
        self._set_home_coordinate(x, y, z, r)

    def home(self):
        return self._extract_cmd_index(self._set_home_cmd())

    def grip(self, enable):
        return self._extract_cmd_index(self._set_end_effector_gripper(enable))

    def laze(self, power=0, enable=False):
        self._set_end_effector_laser(power, enable)

    def speed(self, velocity=100., acceleration=100.):
        self._set_ptp_common_params(velocity, acceleration)
        self._set_ptp_coordinate_params(velocity, acceleration)

    def pose(self):
        response = self._get_pose()
        x = struct.unpack_from('f', response.params, 0)[0]
        y = struct.unpack_from('f', response.params, 4)[0]
        z = struct.unpack_from('f', response.params, 8)[0]
        r = struct.unpack_from('f', response.params, 12)[0]
        j1 = struct.unpack_from('f', response.params, 16)[0]
        j2 = struct.unpack_from('f', response.params, 20)[0]
        j3 = struct.unpack_from('f', response.params, 24)[0]
        j4 = struct.unpack_from('f', response.params, 28)[0]
        return x, y, z, r, j1, j2, j3, j4

    def conveyor_belt(self, speed, direction=1, interface=0):
        if 0.0 <= speed <= 1.0 and (direction == 1 or direction == -1):
            motor_speed = 70 * speed * STEP_PER_CRICLE / MM_PER_CRICLE * direction
            self._set_stepper_motor(motor_speed, interface)
        else:
            raise DobotException("Wrong Parameter")

    def _set_stepper_motor(self, speed, interface=0, motor_control=True):
        msg = Message()
        msg.id = 0x87
        msg.ctrl = 0x03
        msg.params = bytearray([])
        if interface == 1:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        if motor_control is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        msg.params.extend(bytearray(struct.pack('i', speed)))
        return self._send_command(msg)

    def conveyor_belt_distance(self, speed, distance, direction=1, interface=0):
        if 0.0 <= speed <= 100.0 and (direction == 1 or direction == -1):
            motor_speed = speed * STEP_PER_CRICLE / MM_PER_CRICLE * direction
            self._set_stepper_motor_distance(motor_speed, distance, interface)
        else:
            raise DobotException("Wrong Parameter")

    def _set_stepper_motor_distance(self, speed, distance, interface=0, motor_control=True):
        msg = Message()
        msg.id = 0x88
        msg.ctrl = 0x03
        msg.params = bytearray([])
        if interface == 1:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        if motor_control is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        msg.params.extend(bytearray(struct.pack('i', speed)))
        msg.params.extend(bytearray(struct.pack('I', distance)))
        return self._send_command(msg)
