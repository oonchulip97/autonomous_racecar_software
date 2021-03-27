#!/usr/bin/env python

"""ROS Node to teleoperate Autonomous RaceCar."""


import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


class Key(object):
    """Class to hold key mapping to velocity."""

    prompt = """
        Control Your racecar!
        ---------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .

        space key, k : stop
        w/x: shift the middle pos of throttle by +/- 5 pwm
        a/d: shift the middle pos of steering by -/+ 2 deg
        CTRL-C to quit
        """

    move_keys = {
        'u': (1, -1),
        'i': (1, 0),
        'o': (1, 1),
        'j': (0, -1),
        'k': (0, 0),
        'l': (0, 1),
        'm': (-1, -1),
        ',': (-1, 0),
        '.': (-1, 1)}

    bias_keys = {
        'w': (1, 0),
        'x': (-1, 0),
        'a': (0, -1),
        'd': (0, 1)}

    stop_key = ' '

    break_key = chr(3)  # Keycode for Ctrl-C in raw mode

    @staticmethod
    def get_key():
        """Gets keyboard pressed key.

        Blocking. Returns empty string after timeout.

        :returns: string of single character
        :rtype: string
        """
        file_descriptor = sys.stdin.fileno()

        settings = termios.tcgetattr(file_descriptor)
        tty.setraw(file_descriptor)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(file_descriptor, termios.TCSADRAIN, settings)
        return key

    @staticmethod
    def get_key_mapping():
        """Gets key mapping for command velocity.

        :returns: moving key mapping, biasing key mapping,
            stopping key mapping, breaking key mapping
        :rtype: ({str: [int, int], ..}, {str: [int, int], ..}, str, str)
        """
        return (Key.move_keys, Key.bias_keys, Key.stop_key, Key.break_key)

    @staticmethod
    def print_prompt():
        """Prints prompt of key mapping."""
        print(Key.prompt)


class Velocity(object):
    """Class to hold throttling and steering values.

    Throttling value is duration of HIGH state of 50Hz pwm in microseconds.
    Steering value is angle of shaft in degrees.
    """
    # Neutral values which should ensure car is stationary and straight
    speed_mid = 1500
    turn_mid = 90

    # Increment in adjustment of neutral values
    speed_bias_increment = 5
    turn_bias_increment = 2

    # Increment in throttling and steering
    speed_forward = 20
    speed_backward = 40
    turn = 10

    # State machine for reversing
    reverse_pre, reverse_stop, reverse_post = range(3)
    reverse_count_limit = 2

    def __init__(self):
        """Constructor."""
        # Adjustment of neutral values to ensure car is stationary and straight
        self.speed_bias = 0
        self.turn_bias = 0

        # Throttling and steering values
        self.command_speed = Velocity.speed_mid
        self.command_turn = Velocity.turn_mid

        # State of reversing
        self.reverse_state = Velocity.reverse_pre
        self.reverse_count = 0

    def reset_command(self):
        """Resets throttling and steering values so car is stationary and
        straight.
        """
        self.command_speed = Velocity.speed_mid + self.speed_bias
        self.command_turn = Velocity.turn_mid + self.turn_bias

    def change_command(self, speed_dir, turn_dir):
        """Changes throttling and steering value.

        :param speed_dir: positive to move forward, negative to move backward
        :param turn_dir: negative to move left, positive to move right
        :type speed_dir: int
        :type turn_dir: int
        """
        if speed_dir >= 0:
            self.command_speed = (Velocity.speed_forward * speed_dir
                                  + Velocity.speed_mid + self.speed_bias)
            # Reset state for reversing
            self.reverse_state = Velocity.reverse_pre
            self.reverse_count = 0
        else:
            self.reverse(speed_dir)
        self.command_turn = (Velocity.turn * turn_dir + Velocity.turn_mid
                             + self.turn_bias)

    def reverse(self, speed_dir):
        """Reverses the car, which requires slightly complex logic.

        - Send negative throttling value
        - Send zero throttling value
        - Send negative throttling value

        :param speed_dir: negative, magnitude of reversing
        :type speed_dir: int
        """
        # Check whether reversing is required
        if not (speed_dir < 0):
            raise ValueError("auto_racecar_teleop: "
                             + "Speed is not negative for reversing")

        # Change state based on previous state
        if self.reverse_state == Velocity.reverse_pre:
            if self.reverse_count < Velocity.reverse_count_limit:
                self.reverse_count += 1
            else:
                self.reverse_count = 0
                self.reverse_state = Velocity.reverse_stop
        elif self.reverse_state == Velocity.reverse_stop:
            if self.reverse_count < Velocity.reverse_count_limit:
                self.reverse_count += 1
            else:
                self.reverse_count = 0
                self.reverse_state = Velocity.reverse_post
        elif self.reverse_state == Velocity.reverse_post:
            pass
        else:
            raise RuntimeError("auto_racecar_teleop: State is not defined")

        # Output throttling value based on current state
        if self.reverse_state == Velocity.reverse_pre:
            self.command_speed = (Velocity.speed_backward * speed_dir
                                  + Velocity.speed_mid + self.speed_bias)
        elif self.reverse_state == Velocity.reverse_stop:
            self.command_speed = Velocity.speed_mid + self.speed_bias
        elif self.reverse_state == Velocity.reverse_post:
            self.command_speed = (Velocity.speed_backward * speed_dir
                                  + Velocity.speed_mid + self.speed_bias)
        else:
            raise RuntimeError("auto_racecar_teleop: State is not defined")

    def change_bias(self, speed_dir, turn_dir):
        """Changes the neutral throttling and steering value
        so car is stationary and straight.

        :param speed_dir: positive to increase neutral throttling value,
            negative to decrease neutral throttling value
        :param turn_dir: positive to increase neutral steering value,
            negative to decrease neutral steering value
        :type speed_dir: int
        :type turn_dir: int
        """
        speed_bias_diff = (Velocity.speed_bias_increment * speed_dir)
        self.speed_bias += speed_bias_diff
        self.command_speed += speed_bias_diff

        turn_bias_diff = (Velocity.turn_bias_increment * turn_dir)
        self.turn_bias += turn_bias_diff
        self.command_turn += turn_bias_diff

    def get_velocity(self):
        """Returns current throttling and steering values.

        :returns: (throttling value, steering value)
        :rtype: tuple of int
        """
        return (self.command_speed, self.command_turn)

    def print_velocity(self):
        """Prints current throttling and steering values."""
        print("Current:\tSpeed %d\tTurn %d" % (self.command_speed,
                                               self.command_turn))


class Process(object):
    """Class to translate pressed key into command velocity."""
    # How long should moving last
    move_count_limit = 5

    def __init__(self, key, vel):
        """Constructor.

        :param key: Pressed key
        :param vel: Command velocity
        :type key: Key
        :type vel: Velocity
        """
        self.key = key
        self.vel = vel

        # Gets key mapping
        self.move_keys, self.bias_keys, self.stop_key, self.break_key = (
            key.get_key_mapping())

        # Ensure moving is momentary for single key press
        self.move_count = 0

    def process_key(self):
        """Translate pressed key into command velocity."""
        # Gets pressed key
        pressed = self.key.get_key()

        # Move
        if pressed in self.move_keys.keys():
            self.vel.change_command(
                self.move_keys[pressed][0],
                self.move_keys[pressed][1])
            self.move_count = 0
        # Bias
        elif pressed in self.bias_keys.keys():
            self.vel.change_bias(
                self.bias_keys[pressed][0],
                self.bias_keys[pressed][1])
            self.vel.print_velocity()
        # Stop
        elif pressed == self.stop_key:
            self.vel.reset_command()
        # Break
        elif pressed == self.break_key:
            raise rospy.ROSInterruptException
        # Catch-all
        else:
            # Stop after moving
            if self.move_count < Process.move_count_limit:
                self.move_count += 1
            else:
                self.vel.reset_command()


def publish_command(pub, speed, turn):
    """Publish command velocity to the car.

    :param pub: publisher
    :param speed: throttling value in microseconds
    :param turn: steering value in degrees
    :type pub: rospy.Publisher
    :type speed: int
    :type turn: int
    """
    twist = Twist()
    twist.linear.x = speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = turn
    pub.publish(twist)


def main():
    """Main function."""
    rospy.init_node('auto_racecar_teleop')
    pub = rospy.Publisher('car/cmd_vel', Twist, queue_size=5)

    key = Key()
    vel = Velocity()
    process = Process(key, vel)
    try:
        key.print_prompt()
        vel.print_velocity()

        while(True):
            # Process key into command velocity
            process.process_key()

            # Publish command velocity
            command = vel.get_velocity()
            publish_command(pub, command[0], command[1])

    except rospy.ROSInterruptException:
        print("auto_racecar_teleop: Node is shutting down.")

    finally:
        # Stop the car in the end
        vel.reset_command()
        command = vel.get_velocity()
        publish_command(pub, command[0], command[1])


if __name__ == "__main__":
    main()
