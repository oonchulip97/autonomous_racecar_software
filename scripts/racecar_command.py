#!/usr/bin/env python

"""ROS Node to convert command velocity into ESC commands."""

import rospy
from geometry_msgs.msg import Twist


class CommandConverter(object):
    """Class to convert command velocity into ESC commands.

    Steering value is angle of shaft in degrees.
    Throttling value is duration of HIGH state of 50Hz pwm in microseconds.
    """
    steer_map = ((0.08, -0.08), (75, 105))
    throttle_forward_map = ((1, 3), (1575, 1625))
    throttle_backward_map = ((-1, -3), (1425, 1375))
    throttle_stop_value = 1500

    # State machine for reversing
    reverse_pre, reverse_stop, reverse_post = range(3)
    reverse_count_limit = 2

    def __init__(self):
        """Constructor."""
        rospy.Subscriber("cmd_vel", Twist, self.convert_command_callback)

        self.command_publisher = rospy.Publisher('car/cmd_vel', Twist,
                                                 queue_size=1)

        # State of reversing
        self.reverse_state = CommandConverter.reverse_pre
        self.reverse_count = 0

    def convert_command_callback(self, cmd_vel):
        """Convert command velocity into joint commands.

        :param data: command velocity
        :type data: geometry_msgs/Twist
        """
        turn = cmd_vel.angular.z
        steer = range_map(turn,
                          CommandConverter.steer_map[0][0],
                          CommandConverter.steer_map[0][1],
                          CommandConverter.steer_map[1][0],
                          CommandConverter.steer_map[1][1])

        speed = cmd_vel.linear.x
        # Move forward
        if speed > 0:
            throttle = range_map(speed,
                                 CommandConverter.throttle_forward_map[0][0],
                                 CommandConverter.throttle_forward_map[0][1],
                                 CommandConverter.throttle_forward_map[1][0],
                                 CommandConverter.throttle_forward_map[1][1])
            # Reset state for reversing
            self.reverse_state = CommandConverter.reverse_pre
            self.reverse_count = 0
        # Move backward
        elif speed < 0:
            throttle = self.reverse(speed)
        # Stop
        else:
            throttle = CommandConverter.throttle_stop_value

        self.publish_command(steer, throttle)

    def reverse(self, speed):
        """Reverses the car, which requires slightly complex logic.

        - Send negative throttling value
        - Send zero throttling value
        - Send negative throttling value

        :param speed: linear velocity for reversing
        :type speed: float

        :returns: throttling value for reversing
        :rtype: float
        """
        # Check whether reversing is required
        if not speed < 0:
            raise ValueError("racecar_command: "
                             + "Speed is not negative for reversing")

        # Change state based on previous state
        if self.reverse_state == CommandConverter.reverse_pre:
            if self.reverse_count < CommandConverter.reverse_count_limit:
                self.reverse_count += 1
            else:
                self.reverse_count = 0
                self.reverse_state = CommandConverter.reverse_stop
        elif self.reverse_state == CommandConverter.reverse_stop:
            if self.reverse_count < CommandConverter.reverse_count_limit:
                self.reverse_count += 1
            else:
                self.reverse_count = 0
                self.reverse_state = CommandConverter.reverse_post
        elif self.reverse_state == CommandConverter.reverse_post:
            pass
        else:
            raise RuntimeError("racecar_command: State is not defined")

        # Output throttling value based on current state
        if self.reverse_state == CommandConverter.reverse_pre:
            throttle = range_map(speed,
                                 CommandConverter.throttle_backward_map[0][0],
                                 CommandConverter.throttle_backward_map[0][1],
                                 CommandConverter.throttle_backward_map[1][0],
                                 CommandConverter.throttle_backward_map[1][1])
        elif self.reverse_state == CommandConverter.reverse_stop:
            throttle = CommandConverter.throttle_stop_value
        elif self.reverse_state == CommandConverter.reverse_post:
            throttle = range_map(speed,
                                 CommandConverter.throttle_backward_map[0][0],
                                 CommandConverter.throttle_backward_map[0][1],
                                 CommandConverter.throttle_backward_map[1][0],
                                 CommandConverter.throttle_backward_map[1][1])
        else:
            raise RuntimeError("racecar_command: State is not defined")

        return throttle

    def publish_command(self, steer, throttle):
        """Publish ESC command.

        :param steer: steering value in degrees
        :param throttle: throttling value in microseconds
        :type steer: float
        :type throttle: float
        """
        twist = Twist()
        twist.linear.x = throttle
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = steer
        self.command_publisher.publish(twist)


def range_map(x, from_low, from_high, to_low, to_high, restrict=True):
    """Remaps a value from one range to another.

    That is, value of from_low will be mapped to to_low,
    value of from_high will be mapped to to_high,
    values in-between to values in-between.

    Can also constraint value within the new range.

    :param x: value to be mapped
    :param from_low: lower threshold of previous range
    :param from_high: higher threshold of previous range
    :param to_low: lower threshold of new range
    :param to_high: higher threshold of new range
    :param restrict: whether to constraint new value within new range
    :type x: float
    :type from_low: float
    :type from_high: float
    :type to_low: float
    :type to_high: float
    :type restrict: bool

    :returns: value that is mapped
    :rtype: float
    """
    # Linear mapping
    y = ((to_high - to_low) / (from_high - from_low)) * (x - from_low) + to_low

    if restrict:
        if to_high >= to_low:
            y = max(to_low, min(to_high, y))
        else:
            y = max(to_high, min(to_low, y))

    return y


def main():
    """Main function."""
    rospy.init_node('racecar_command')
    _ = CommandConverter()
    rospy.spin()


if __name__ == '__main__':
    main()
