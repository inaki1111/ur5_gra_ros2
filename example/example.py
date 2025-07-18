#!/usr/bin/env python3
import rclpy
import time
import pprint
import socket
import threading
from enum import Enum
from typing import Union, Tuple, OrderedDict
from ur5e_moveit_client import UR5eMoveItClient


class GripperStatus(Enum):
    RESET = 0
    ACTIVATING = 1
    ACTIVE = 3


class ObjectStatus(Enum):
    MOVING = 0
    STOPPED_OUTER_OBJECT = 1
    STOPPED_INNER_OBJECT = 2
    AT_DEST = 3


class RobotiqGripper:
    ACT = 'ACT'
    GTO = 'GTO'
    ATR = 'ATR'
    ADR = 'ADR'
    FOR = 'FOR'
    SPE = 'SPE'
    POS = 'POS'
    STA = 'STA'
    PRE = 'PRE'
    OBJ = 'OBJ'
    FLT = 'FLT'
    ENCODING = 'UTF-8'
    IP = '192.168.0.2'

    def __init__(self):
        self.socket = None
        self.command_lock = threading.Lock()
        self._min_position = 0
        self._max_position = 255
        self._min_speed = 0
        self._max_speed = 255
        self._min_force = 0
        self._max_force = 255

    def connect(self, hostname: str, port: int, socket_timeout: float = 2.0) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((hostname, port))
        self.socket.settimeout(socket_timeout)

    def disconnect(self) -> None:
        self.socket.close()

    def _set_vars(self, var_dict: OrderedDict[str, Union[int, float]]):
        cmd = "SET"
        for variable, value in var_dict.items():
            cmd += f" {variable} {str(value)}"
        cmd += '\n'
        with self.command_lock:
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        return self._is_ack(data)

    def _set_var(self, variable: str, value: Union[int, float]):
        return self._set_vars(OrderedDict([(variable, value)]))

    def _get_var(self, variable: str):
        with self.command_lock:
            cmd = f"GET {variable}\n"
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError(f"Unexpected response {data} ({data.decode(self.ENCODING)}): does not match '{variable}'")
        return int(value_str)

    @staticmethod
    def _is_ack(data: str):
        return data == b'ack'

    def _reset(self):
        self._set_var(self.ACT, 0)
        self._set_var(self.ATR, 0)
        while not (self._get_var(self.ACT) == 0 and self._get_var(self.STA) == 0):
            self._set_var(self.ACT, 0)
            self._set_var(self.ATR, 0)
        time.sleep(0.5)

    def activate(self, auto_calibrate: bool = False):
        if not self.is_active():
            self._reset()
            while not (self._get_var(self.ACT) == 0 and self._get_var(self.STA) == 0):
                time.sleep(0.01)
            self._set_var(self.ACT, 1)
            time.sleep(1.0)
            while not (self._get_var(self.ACT) == 1 and self._get_var(self.STA) == 3):
                time.sleep(0.01)
        if auto_calibrate:
            self.auto_calibrate()

    def is_active(self):
        return GripperStatus(self._get_var(self.STA)) == GripperStatus.ACTIVE

    def get_min_position(self) -> int:
        return self._min_position

    def get_max_position(self) -> int:
        return self._max_position

    def get_open_position(self) -> int:
        return self.get_min_position()

    def get_closed_position(self) -> int:
        return self.get_max_position()

    def is_open(self):
        return self.get_current_position() <= self.get_open_position()

    def is_closed(self):
        return self.get_current_position() >= self.get_closed_position()

    def get_current_position(self) -> int:
        return self._get_var(self.POS)

    def auto_calibrate(self, log: bool = True) -> None:
        (position, status) = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed opening to start: {str(status)}")
        (position, status) = self.move_and_wait_for_pos(self.get_closed_position(), 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed because of an object: {str(status)}")
        assert position <= self._max_position
        self._max_position = position
        (position, status) = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed because of an object: {str(status)}")
        assert position >= self._min_position
        self._min_position = position
        if log:
            print(f"Gripper auto-calibrated to [{self.get_min_position()}, {self.get_max_position()}]")

    def move(self, position: int, speed: int, force: int) -> Tuple[bool, int]:
        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))
        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)
        var_dict = OrderedDict([(self.POS, clip_pos), (self.SPE, clip_spe), (self.FOR, clip_for), (self.GTO, 1)])
        return self._set_vars(var_dict), clip_pos

    def move_and_wait_for_pos(self, position: int, speed: int, force: int) -> Tuple[int, ObjectStatus]:
        set_ok, cmd_pos = self.move(position, speed, force)
        if not set_ok:
            raise RuntimeError("Failed to set variables for move.")
        while self._get_var(self.PRE) != cmd_pos:
            time.sleep(0.001)
        cur_obj = self._get_var(self.OBJ)
        while ObjectStatus(cur_obj) == ObjectStatus.MOVING:
            cur_obj = self._get_var(self.OBJ)
        final_pos = self._get_var(self.POS)
        final_obj = cur_obj
        return final_pos, ObjectStatus(final_obj)

    def get_status(self) -> Tuple[GripperStatus, ObjectStatus, int]:
        return (GripperStatus(self._get_var(self.STA)),
                ObjectStatus(self._get_var(self.OBJ)),
                self._get_var(self.FLT))


def main():
    print('initializing node...')
    rclpy.init()
    node = UR5eMoveItClient()
    pp = pprint.PrettyPrinter(indent=4)
    gripper = RobotiqGripper()
    gripper.connect(gripper.IP, 63352)
    gripper.activate()

    poses = {
        'pose0': [-0.2, 0.7, 0.5, 1.0, 0.0, 0.0, 0.0],
        'pose1': [-0.2, 0.7, 0.2, 1.0, 0.0, 0.0, 0.0],
        'pose2': [-0.2, 0.7, 0.5, 1.0, 0.0, 0.0, 0.0],
        'pose3': [0.2, 0.7, 0.5, 1.0, 0.0, 0.0, 0.0],
        'pose4': [0.2, 0.7, 0.2, 1.0, 0.0, 0.0, 0.0],
        'pose5': [0.0, 0.3, 0.5, 1.0, 0.0, 0.0, 0.0],
    }

    print('Starting joint values and tool position')
    jp = node.get_joint_positions()
    pp.pprint(list(jp))

    link_name = 'tool0'
    position, orientation = node.get_link_position(link_name)
    if position and orientation:
        print(f'Position of {link_name}: {position}')
        print(f'Orientation of {link_name}: {orientation}')

    node.move_to_joints([1.57] + list(jp[1:]))

    jp = node.get_joint_positions()
    pp.pprint(list(jp))

    for pose_name, pose in poses.items():
        node.move_to_pose(pose)
        print(f'{pose_name} done')
        time.sleep(1)
        if pose_name == 'pose1':
            gripper.move_and_wait_for_pos(255, 255, 255)
        if pose_name == 'pose4':
            gripper.move_and_wait_for_pos(0, 255, 255)

    print('Ending joint values and tool position')
    jp = node.get_joint_positions()
    pp.pprint(list(jp))
    position, orientation = node.get_link_position('tool0')
    if position and orientation:
        print(f'Position of tool0: {position}')
        print(f'Orientation of tool0: {orientation}')

    gripper.disconnect()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
