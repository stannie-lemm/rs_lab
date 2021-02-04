import numpy as np 
from can import CAN_Bus
from time import time
from typing import NoReturn, Optional, Union, List
from time import sleep

class CAN_Device:
    dev_id: int
    bus: CAN_Bus 

    def __init__(self, dev_id: int, bus: CAN_Bus):
        self.dev_id = dev_id
        self.bus = bus
    
    def send(self, data: bytes) -> None:
        if not isinstance(data, bytes):
            raise TypeError('Data type must be bytes')
        self.bus.send_bytes(self.dev_id, data)
    
    def recive(self) -> bytes:
        can_id, _, data = self.bus.recive_frame()
        if can_id == self.dev_id:
            return data
    

class CAN_Motor(CAN_Device):
    current_position: Optional[int] = None
    current_positions: List[float] = [0 for i in range(20)]
    current_turns: int = 0
    current_vel: float = 0.0
    current_time: float = 0
    current_data: Optional[bytes] = None
    hi: int = 2**14
    lo: int = 0
    _rad_mul = np.pi*2/2**14
    _deg_mul = 360/2**14
    MOTOR_STATUS_BASE: bytes = b'\x9C' + 7*b'\x00'
    TORQUE_CONTROL_BASE: bytes = b'\xA1'
    MOTOR_OFF_BASE: bytes = b'\x80' + 7*b'\x00'
    current_torque: int = 0

    def _generate_torque_frame(self, val: int):
        if val < -2000 or val > 2000:
            raise ValueError('Torque should be in [-2000; 2000] range')
        val_b = val.to_bytes(2, 'little', signed=True)
        temp = self.TORQUE_CONTROL_BASE + 3*b'\x00' + val_b + 2*b'\x00'
        # temp[4] = val_b[0]
        # temp[5] = val_b[1]
        return temp
    
    def _send_n_rcv_torque(self, val: int):
        self.current_torque = val
        frame = self._generate_torque_frame(val)
        self.send(frame)
        self.recv_data()
    
    def smooth_off(self):
        torques = np.linspace(self.current_torque, 0, 5, dtype=np.int16)
        for t in torques:
            self._send_n_rcv_torque(int(t))
            sleep(0.1)
        while np.abs(self.current_vel) > 1:
            self.recv_data()
            self.get_inst_vel()
    
    def motor_off(self):
        self.send(self.MOTOR_OFF_BASE)
        
    def _calc_radians(self, val) -> float:
        return val*self._rad_mul
    
    def _calc_degrees(self, val) -> float:
        return val*self._deg_mul

    def _calc_abs_pos(self) -> int:
        return self.hi*self.current_turns + self.current_position
    
    def recv_data(self, send: bool=True) -> None:
        if send:
            self.send(self.MOTOR_STATUS_BASE)
        self.current_data = self.recive()
    
        
    def _recv_pos(self, threshold_lo: int=2**4, threshold_hi: int = 2**12) -> None:
        # self.current_time = time()
        data = self.current_data
        if data is None:
            return

        parsed = int.from_bytes(data[6:],byteorder = 'little')
        if self.current_position is None:
            self.current_position = parsed
        elif self.current_position > threshold_hi and parsed <= threshold_lo:
            self.current_turns += 1
        elif self.current_position <= threshold_lo and parsed > threshold_hi:
            self.current_turns -= 1
        self.current_position = parsed
        self.current_positions.pop(0)
        self.current_positions.append(self._calc_degrees(self.current_position))

    def get_position(self, units: Optional[str] = None) -> Union[float, int]:
        self._recv_pos()
        pos = self._calc_abs_pos()
        if units is None:
            return pos
        elif units is 'rad':
            return self._calc_radians(pos)
        elif units is 'deg':
            return self._calc_degrees(pos)
        else: 
            raise ValueError(f'No such unit {units}')

    def get_inst_vel(self) -> Optional[float]:
        data = self.current_data
        if data is None:
            return

        parsed = int.from_bytes(data[4:6],byteorder = 'little', signed=True)
        self.current_vel = parsed
        return self.current_vel

    def get_inst_vel_calc(self, threshold_lo: int=2**4, threshold_hi: int = 2**12) -> Optional[float]:
        self.send(self.MOTOR_STATUS_BASE)
        data = self.recive()
        dt = time() - self.current_time
        if data is None:
            return

        parsed = int.from_bytes(data[6:],byteorder = 'little')
        ds = parsed - self.current_position
        return ds/dt

    def get_inst_vel_filtered(self, threshold_lo: int=2**4, threshold_hi: int = 2**12) -> Optional[float]:
        self.recv_data()
        self._recv_pos()
        return np.mean(self.current_positions)
    
    
