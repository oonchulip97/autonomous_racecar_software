#!/usr/bin/env python

"""ROS Node to convert command velocity into joint commands."""

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class CommandConverter(object):
    """Class to convert command velocity into joint commands."""
    steer_multiplier = 5.0  # increased to test planner performance
    throttle_multiplier = 1.0 / 0.0332  # 1 / wheel_radius

    def __init__(self):
        """Constructor."""
        rospy.Subscriber("cmd_vel", Twist, self.convert_command_callback)

        self.steering_hinge_position_publisher = rospy.Publisher(
            '/steering_hinge_position_controller/command', Float64MultiArray,
            queue_size=1)
        self.wheel_velocity_publisher = rospy.Publisher(
            '/wheel_velocity_controller/command', Float64MultiArray,
            queue_size=1)

    def convert_command_callback(self, cmd_vel):
        """Convert command velocity into joint commands.

        :param data: command velocity
        :type data: geometry_msgs/Twist
        """
        steer = cmd_vel.angular.z * CommandConverter.steer_multiplier
        throttle = cmd_vel.linear.x * CommandConverter.throttle_multiplier
        self.publish_command(steer, throttle)

    def publish_command(self, steer, throttle):
        """Publish joint commands.

        :param steer: steering angle
        :param throttle: throttle velocity
        :type steer: float
        :type throttle: float
        """
        steer_array = Float64MultiArray()
        steer_array.data = [steer, steer]
        self.steering_hinge_position_publisher.publish(steer_array)

        throttle_array = Float64MultiArray()
        throttle_array.data = [throttle, throttle, throttle, throttle]
        self.wheel_velocity_publisher.publish(throttle_array)


def main():
    """Main function."""
    rospy.init_node('gazebo_command')
    _ = CommandConverter()
    rospy.spin()


if __name__ == '__main__':
    main()
