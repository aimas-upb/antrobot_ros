# Copyright (C) 2021 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from smbus2 import SMBus
import struct


class RDrive:
    __COM_EN_DIS = 0
    __COM_MOVE = 1

    def __init__(
            self, pose=None, wheel_diameter=0.04, wheel_separation=0.135,
            has_encoder=True, encoder_resolution=1800,
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
        self.wheel_diameter = wheel_diameter
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
                bus.write_i2c_block_data(self.i2c_addr, self.__COM_EN_DIS, [1])
                self.drive_enable = True
            except IOError:
                print("enable(): IOError in i2c write.")
                self.drive_enable = False
        return self.drive_enable

    def disable(self):
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__COM_EN_DIS, [0])
                self.drive_enable = False
            except IOError:
                print("disable(): IOError in i2c write.")
        return not self.drive_enable

    def move(self, v, omega):
        msg = [byte for data_item in [v, omega] for byte in struct.pack('<f', data_item)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__COM_MOVE, msg)
                self.linear_velocity = v
                self.angular_velocity = omega
            except IOError:
                print("move(): IOError in i2c write.")
