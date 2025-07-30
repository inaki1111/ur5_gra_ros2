#!/usr/bin/env python3
import socket
import threading
import time
from enum import Enum
from typing import Union, Tuple, OrderedDict

class RobotiqGripper:
    # Códigos de comando
    ACT = 'ACT'; GTO = 'GTO'; ATR = 'ATR'; ADR = 'ADR'
    FOR = 'FOR'; SPE = 'SPE'; POS = 'POS'; STA = 'STA'
    PRE = 'PRE'; OBJ = 'OBJ'; FLT = 'FLT'
    ENCODING = 'UTF-8'
    IP = '192.168.0.2'
    PORT = 63352

    def __init__(self):
        self.socket = None
        self.command_lock = threading.Lock()
        self._min_position = 0
        self._max_position = 255
        self._min_speed = 0
        self._max_speed = 255
        self._min_force = 0
        self._max_force = 255

    class GripperStatus(Enum):
        RESET = 0
        ACTIVATING = 1
        ACTIVE = 3

    class ObjectStatus(Enum):
        MOVING = 0
        STOPPED_OUTER_OBJECT = 1
        STOPPED_INNER_OBJECT = 2
        AT_DEST = 3

    def connect(self, hostname: str=IP, port: int=PORT, timeout: float=2.0) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((hostname, port))
        self.socket.settimeout(timeout)

    def disconnect(self) -> None:
        if self.socket:
            self.socket.close()

    def _set_vars(self, var_dict: OrderedDict[str, Union[int, float]]):
        cmd = "SET " + " ".join(f"{v} {val}" for v, val in var_dict.items()) + "\n"
        with self.command_lock:
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        return data == b'ack'

    def _get_var(self, variable: str) -> int:
        with self.command_lock:
            self.socket.sendall(f"GET {variable}\n".encode(self.ENCODING))
            data = self.socket.recv(1024).decode(self.ENCODING).split()
        if data[0] != variable:
            raise ValueError(f"Error: {data}")
        return int(data[1])

    def _reset(self):
        self._set_vars(OrderedDict([(self.ACT, 0), (self.ATR, 0)]))
        while self._get_var(self.ACT) != 0 or self._get_var(self.STA) != 0:
            self._set_vars(OrderedDict([(self.ACT, 0), (self.ATR, 0)]))
        time.sleep(0.5)

    def activate(self, auto_calibrate: bool=False):
        if not self.is_active():
            self._reset()
            while self._get_var(self.ACT) != 0 or self._get_var(self.STA) != 0:
                time.sleep(0.01)
            self._set_vars(OrderedDict([(self.ACT, 1)]))
            time.sleep(1.0)
            while self._get_var(self.ACT) != 1 or self._get_var(self.STA) != 3:
                time.sleep(0.01)
        if auto_calibrate:
            self.auto_calibrate()

    def is_active(self) -> bool:
        return RobotiqGripper.GripperStatus(self._get_var(self.STA)) \
               == RobotiqGripper.GripperStatus.ACTIVE

    def get_current_position(self) -> int:
        return self._get_var(self.POS)

    def move(self, position: int, speed: int, force: int) -> Tuple[bool, int]:
        clip = lambda mn, x, mx: max(mn, min(x, mx))
        p = clip(self._min_position, position, self._max_position)
        s = clip(self._min_speed, speed, self._max_speed)
        f = clip(self._min_force, force, self._max_force)
        ok = self._set_vars(OrderedDict([
            (self.POS, p), (self.SPE, s), (self.FOR, f), (self.GTO, 1)
        ]))
        return ok, p

    def move_and_wait_for_pos(self, position: int, speed: int, force: int) \
            -> Tuple[int, ObjectStatus]:
        ok, cmd_pos = self.move(position, speed, force)
        if not ok:
            raise RuntimeError("No se pudieron fijar variables de movimiento")
        while self._get_var(self.PRE) != cmd_pos:
            time.sleep(0.001)
        status = self._get_var(self.OBJ)
        while RobotiqGripper.ObjectStatus(status) == RobotiqGripper.ObjectStatus.MOVING:
            status = self._get_var(self.OBJ)
        return self._get_var(self.POS), RobotiqGripper.ObjectStatus(status)

    def auto_calibrate(self, log: bool=True):
        pos, st = self.move_and_wait_for_pos(self._min_position, 64, 1)
        if st != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError("Fallo calibración apertura")
        pos, st = self.move_and_wait_for_pos(self._max_position, 64, 1)
        if st != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError("Fallo calibración cierre")
        self._max_position = pos
        pos, st = self.move_and_wait_for_pos(self._min_position, 64, 1)
        if st != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError("Fallo calibración apertura final")
        self._min_position = pos
        if log:
            print(f"Pinza calibrada: [{self._min_position}, {self._max_position}]")
