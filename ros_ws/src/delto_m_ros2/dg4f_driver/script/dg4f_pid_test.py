#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand
import time
import math


class PIDControlTest(Node):
    def __init__(self):
        super().__init__('pid_control_test')

        # Create publishers for each joint PID controller
        self.joint_publishers = {}
        self.joint_names = [
            'j_dg_1_1', 'j_dg_1_2', 'j_dg_1_3', 'j_dg_1_4',
            'j_dg_2_1', 'j_dg_2_2', 'j_dg_2_3', 'j_dg_2_4',
            'j_dg_3_1', 'j_dg_3_2', 'j_dg_3_3', 'j_dg_3_4',
            'j_dg_4_1', 'j_dg_4_2', 'j_dg_4_3', 'j_dg_4_4',
            'j_dg_1_inner', 'j_dg_4_inner'
        ]

        for joint_name in self.joint_names:
            topic_name = f'/dg4f/{joint_name}_pospid/reference'
            self.joint_publishers[joint_name] = self.create_publisher(
                MultiDOFCommand,
                topic_name,
                10
            )
            self.get_logger().info(f'Created publisher for {topic_name}')

        # Timer for sending commands
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.start_time = time.time()
        self.current_joint_index = 0
        self.movement_duration = 2.0  # seconds per joint
        self.last_switch_time = time.time()

    def timer_callback(self):
        current_time = time.time()
        elapsed = current_time - self.start_time

        # Switch to next joint every movement_duration seconds
        if current_time - self.last_switch_time >= self.movement_duration:
            self.current_joint_index = (
                self.current_joint_index + 1) % len(self.joint_names)
            self.last_switch_time = current_time
            self.get_logger().info(
                f'Now controlling: {self.joint_names[self.current_joint_index]}')

        # Create sinusoidal reference signal for current joint
        amplitude = 0.3  # radians
        frequency = 0.5  # Hz
        position = amplitude * math.sin(2 * math.pi * frequency * elapsed)

        # Send command to current joint
        msg = MultiDOFCommand()
        msg.dof_names = [current_joint]
        msg.values = [position]
        msg.values_dot = [0.0]

        current_joint = self.joint_names[self.current_joint_index]
        self.joint_publishers[current_joint].publish(msg)

        # Send zero to all other joints to hold position
        zero_msg = MultiDOFCommand()

        for i, joint_name in enumerate(self.joint_names):
            if i != self.current_joint_index:
                zero_msg.dof_names = [joint_name]
                zero_msg.values = [0.0]
                zero_msg.values_dot = [0.0]
                self.joint_publishers[joint_name].publish(zero_msg)

        # Log current command
        if int(elapsed * 10) % 10 == 0:  # Log every second
            self.get_logger().info(
                f'Joint {current_joint}: position = {position:.3f} rad'
            )


def main(args=None):
    rclpy.init(args=args)

    node = PIDControlTest()

    try:
        node.get_logger().info('Starting PID control test...')
        node.get_logger().info('Each joint will move in sequence with sinusoidal motion')
        node.get_logger().info('Press Ctrl+C to stop')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
