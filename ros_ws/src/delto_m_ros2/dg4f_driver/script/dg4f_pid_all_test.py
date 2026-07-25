#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand
import time
import math


class PIDControlTestAll(Node):
    def __init__(self):
        super().__init__('pid_control_test_all')

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
        # 100Hz for smoother control
        self.timer = self.create_timer(0.005, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        current_time = time.time()
        elapsed = current_time - self.start_time

        # Different motion patterns for different joints
        for i, joint_name in enumerate(self.joint_names):
            msg = MultiDOFCommand()
            msg.dof_names = [joint_name]

            if 'inner' in joint_name:
                # Inner joints: slower, smaller amplitude
                amplitude = 0.3
                frequency = 0.1
                phase = 0
            if 'j_dg_2_4' in joint_name:
                amplitude = 1.00
                frequency = 1
                phase = 0
            else:
                amplitude = 0.0
                frequency = 0.01
                phase = 0

            # Generate sinusoidal position command
            position = amplitude * \
                (math.sin(2 * math.pi * frequency * elapsed + phase))

            msg.values = [position]
            msg.values_dot = [0.0]

            self.joint_publishers[joint_name].publish(msg)
            self.get_logger().info(
                f'Time: {elapsed:.1f}s - All joints moving with sinusoidal patterns')
        # Log status every 2 seconds
        if int(elapsed * 0.5) % 1 == 0 and abs(elapsed - int(elapsed)) < 0.02:
            self.get_logger().info(
                f'Time: {elapsed:.1f}s - All joints moving with sinusoidal patterns')


def main(args=None):
    rclpy.init(args=args)

    node = PIDControlTestAll()

    try:
        node.get_logger().info('Starting PID control test for ALL joints...')
        node.get_logger().info('All joints will move simultaneously with different phases')
        node.get_logger().info('Press Ctrl+C to stop')
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send zero commands to all joints before stopping
        node.get_logger().info('Stopping all joints...')
        for joint_name in node.joint_names:
            zero_msg = MultiDOFCommand()
            zero_msg.dof_names = [joint_name]
            zero_msg.values = [0.0]
            zero_msg.values_dot = [0.0]
            node.joint_publishers[joint_name].publish(zero_msg)
        time.sleep(0.5)
        node.get_logger().info('Test stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
