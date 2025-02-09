#!/usr/bin/env python3
# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import os
import select
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios


class KeyboardTeleoperation(Node):
    def __init__(self):
        super().__init__('keyboard_teleoperation')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('key_mapping.forward', ['w']),
                ('key_mapping.backward', ['s']),
                ('key_mapping.left', ['a']),
                ('key_mapping.right', ['d']),
                ('key_mapping.stop', [' ']),
                ('max_linear_velocity', 0.2),
                ('max_angular_velocity', 1.0),
                ('linear_step_size', 0.05),
                ('angular_step_size', 0.2),
                ('target_linear_velocity', 0.0),
                ('target_angular_velocity', 0.0),
            ]
        )

        self.key_mapping = {
            "forward": self.get_parameter('key_mapping.forward').get_parameter_value().string_array_value,
            "backward": self.get_parameter('key_mapping.backward').get_parameter_value().string_array_value,
            "left": self.get_parameter('key_mapping.left').get_parameter_value().string_array_value,
            "right": self.get_parameter('key_mapping.right').get_parameter_value().string_array_value,
            "stop": self.get_parameter('key_mapping.stop').get_parameter_value().string_array_value,
        }

        self.max_linear_velocity = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.linear_step_size = self.get_parameter('linear_step_size').get_parameter_value().double_value
        self.angular_step_size = self.get_parameter('angular_step_size').get_parameter_value().double_value
        self.target_linear_velocity = self.get_parameter('target_linear_velocity').get_parameter_value().double_value
        self.target_angular_velocity = self.get_parameter('target_angular_velocity').get_parameter_value().double_value

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.usage_msg = (
            "\n"
            "Control Ant Robot!\n"
            "------------------\n"
            "Moving around:\n\n"
            "        " + str('forward') + "\n\n"
            "   " + str('left') + "         " + str('right') + "\n\n"
            "        " + str('backward') + "\n"
            "         " + str('stop') + "\n\n"
            "Keyboard mappings:\n\n"
            "\t" + str('forward') + ": " + str(self.key_mapping['forward']) + " - increase linear velocity \n"
            "\t" + str('backward') + ": " + str(self.key_mapping['backward']) + " - decrease linear velocity \n"
            "\t" + str('left') + ": " + str(self.key_mapping['left']) + " - increase angular velocity \n"
            "\t" + str('right') + ": " + str(self.key_mapping['right']) + " - decrease angular velocity\n"
            "\t" + str('stop') + ": " + str(self.key_mapping['stop']) + " - force stop\n"
            "\tCTRL-C to quit\n"
        )

        self.get_logger().info(self.usage_msg)

        # Terminal settings for non-blocking key input
        self.settings = None
        if os.name != 'nt' and os.isatty(sys.stdin.fileno()):
            self.settings = termios.tcgetattr(sys.stdin)
            self.get_logger().info("Terminal settings retrieved successfully.")
        else:
            self.get_logger().warn("Warning: stdin is not a TTY. Skipping terminal settings.")

        # Thread handling key input
        self.running = True
        self.key_thread = threading.Thread(target=self.read_keys)
        self.key_thread.start()

        # Timer for publishing velocities
        self.create_timer(timer_period_sec=1.0, callback=self.__timer_callback)

    @staticmethod
    def __threshold__(target, low_limit, high_limit):
        return max(low_limit, min(high_limit, target))

    def read_keys(self):
        """Read keys in a separate thread and adjust velocities."""
        try:
            while self.running and rclpy.ok():
                key = self.__get_key__()
                if key:
                    self.process_key(key)
        except Exception as e:
            self.get_logger().error(f"Error in key reading: {e}")

    def __get_key__(self):
        """Read a key press non-blockingly."""
        if os.name == 'nt':
            return msvcrt.getch().decode() if sys.version_info[0] >= 3 else msvcrt.getch()

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key_pressed = sys.stdin.read(1)
        else:
            key_pressed = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key_pressed

    def process_key(self, key):
        """Process the key and update velocities."""
        if key in self.key_mapping['forward']:
            self.target_linear_velocity = self.__threshold__(
                self.target_linear_velocity + self.linear_step_size,
                -self.max_linear_velocity, self.max_linear_velocity
            )
        elif key in self.key_mapping['backward']:
            self.target_linear_velocity = self.__threshold__(
                self.target_linear_velocity - self.linear_step_size,
                -self.max_linear_velocity, self.max_linear_velocity
            )
        elif key in self.key_mapping['left']:
            self.target_angular_velocity = self.__threshold__(
                self.target_angular_velocity + self.angular_step_size,
                -self.max_angular_velocity, self.max_angular_velocity
            )
        elif key in self.key_mapping['right']:
            self.target_angular_velocity = self.__threshold__(
                self.target_angular_velocity - self.angular_step_size,
                -self.max_angular_velocity, self.max_angular_velocity
            )
        elif key in self.key_mapping['stop']:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0
        elif key == '\x03':  # Ctrl-C to quit
            self.running = False
            rclpy.shutdown()

        # Clear the previous velocity message and print the new one
        sys.stdout.write("\033[F\033[K")  # Move cursor up one line and clear the line
        sys.stdout.write(f"Currently: linear velocity {self.target_linear_velocity}, angular velocity {self.target_angular_velocity}\n")
        sys.stdout.flush()

    def __velocities_string__(self):
        return f"Currently: linear velocity {self.target_linear_velocity}, angular velocity {self.target_angular_velocity}"

    def __timer_callback(self):
        """Timer callback to publish velocities."""
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.target_linear_velocity
        cmd_vel_msg.angular.z = self.target_angular_velocity
        self.pub_cmd_vel.publish(cmd_vel_msg)
        self.get_logger().debug(f'Published velocities: {self.target_linear_velocity}, {self.target_angular_velocity}')

    def destroy_node(self):
        """Clean up resources when shutting down."""
        self.running = False
        self.key_thread.join()  # Wait for the key thread to finish

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    keyboard_teleoperation_node = KeyboardTeleoperation()

    try:
        rclpy.spin(keyboard_teleoperation_node)
    except KeyboardInterrupt:
        keyboard_teleoperation_node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        keyboard_teleoperation_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
