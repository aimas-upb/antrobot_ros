# Copyright (C) 2021 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from smbus2 import SMBus
import struct


class RDrive:
    __ENC_CPR_REG = 13
    __DD_EN_DIS = 15
    __DD_WHEEL_SEPARATION = 16
    __DD_WHEEL_RADIUS = 17
    __DD_PID_GAINS = 20
    __DD_VEL_CMD = 21

    def __init__(
            self, pose=None, wheel_diameter=0.06, wheel_separation=0.1345,
            has_encoder=True, encoder_resolution=2100,
            i2c_port=1, i2c_addr=0x70
    ):
        # Set 2D robot pose
        if pose is None:
            pose = [0.0, 0.0, 0.0]
        self.pose = pose

        # Set i2c com rpi to drive parameters
        self.i2c_port = i2c_port
        self.i2c_addr = i2c_addr

        # Set drive mechanical parameters
        self.wheel_radius = wheel_diameter/2.0
        self.wheel_separation = wheel_separation

        # Set drive sensors parameters
        self.has_encoder = has_encoder
        self.encoder_resolution = encoder_resolution

        # Set drive status
        self.drive_enable = False
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def enable(self):
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_EN_DIS, [1])
                self.drive_enable = True
            except IOError:
                print("enable(): IOError in i2c write.")
                self.drive_enable = False
        return self.drive_enable

    def disable(self):
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_EN_DIS, [0])
                self.drive_enable = False
            except IOError:
                print("disable(): IOError in i2c write.")
        return not self.drive_enable

    def cmd_vel(self, v, omega):
        msg = [byte for data_item in [v, omega] for byte in struct.pack('<f', data_item)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_VEL_CMD, msg)
            except IOError:
                print("cmd_vel(): IOError in i2c write.")
            self.linear_velocity = v
            self.angular_velocity = omega

    def set_wheel_radius(self, wheel_radius):
        msg = [byte for byte in struct.pack('<f', wheel_radius)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_WHEEL_RADIUS, msg)
            except IOError:
                print("set_wheel_radius(): IOError in i2c write.")
        self.wheel_radius = wheel_radius

    def set_wheel_separation(self, wheel_separation):
        msg = [byte for byte in struct.pack('<f', wheel_separation)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_WHEEL_SEPARATION, msg)
            except IOError:
                print("set_wheel_separation(): IOError in i2c write.")
        self.wheel_separation = wheel_separation

    def set_pid_gains(self, motor, kp, ki, kd):
        msg = [motor]
        msg += [byte for data_item in [kp, ki, kd] for byte in struct.pack('<f', data_item)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_PID_GAINS, msg)
            except IOError:
                print("set_pid_gains(): IOError in i2c write.")

    def set_encoder_cpr(self, enc, cpr):
        msg = [enc]
        msg += [byte for byte in struct.pack('<i', cpr)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__ENC_CPR_REG, msg)
            except IOError:
                print("set_pid_gains(): IOError in i2c write.")
