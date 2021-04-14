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
        CTRL-C to quit
        """

    move_keys = {
        'u': (1, 1),
        'i': (1, 0),
        'o': (1, -1),
        'j': (0, 1),
        'k': (0, 0),
        'l': (0, -1),
        'm': (-1, 1),
        ',': (-1, 0),
        '.': (-1, -1)}

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

        :returns: moving key mapping, stopping key mapping,
            breaking key mapping
        :rtype: ({str: [int, int], ..}, str, str)
        """
        return (Key.move_keys, Key.stop_key, Key.break_key)

    @staticmethod
    def print_prompt():
        """Prints prompt of key mapping."""
        print(Key.prompt)


class Velocity(object):
    """Class to hold command velocity."""

    # Neutral values which should ensure car is stationary and straight
    speed_mid = 0
    turn_mid = 0

    # Increment in throttling and steering
    speed_inc = 1.5
    turn_inc = 0.08

    def __init__(self):
        """Constructor."""
        # Throttling and steering values
        self.command_speed = Velocity.speed_mid
        self.command_turn = Velocity.turn_mid

    def reset_command(self):
        """Resets throttling and steering values so car is stationary and
        straight.
        """
        self.command_speed = Velocity.speed_mid
        self.command_turn = Velocity.turn_mid

    def change_command(self, speed_dir, turn_dir):
        """Changes throttling and steering value.

        :param speed_dir: positive to move forward, negative to move backward
        :param turn_dir: positive to move left, negative to move right
        :type speed_dir: int
        :type turn_dir: int
        """
        self.command_speed = Velocity.speed_inc * speed_dir + Velocity.speed_mid
        self.command_turn = Velocity.turn_inc * turn_dir + Velocity.turn_mid

    def get_velocity(self):
        """Returns current throttling and steering values.

        :returns: (throttling value, steering value)
        :rtype: tuple of int
        """
        return (self.command_speed, self.command_turn)

    def print_velocity(self):
        """Prints current throttling and steering values."""
        print("Current:\tSpeed %d Turn %d" % (self.command_speed,
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
        self.move_keys, self.stop_key, self.break_key = key.get_key_mapping()

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
    :param speed: throttling value
    :param turn: steering value
    :type pub: rospy.Publisher
    :type speed: float
    :type turn: float
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
    rospy.init_node('racecar_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    key = Key()
    vel = Velocity()
    process = Process(key, vel)
    try:
        key.print_prompt()

        while True:
            # Process key into command velocity
            process.process_key()

            # Publish command velocity
            command = vel.get_velocity()
            publish_command(pub, command[0], command[1])

    except rospy.ROSInterruptException:
        print("racecar_teleop: Node is shutting down.")

    finally:
        # Stop the car in the end
        vel.reset_command()
        command = vel.get_velocity()
        publish_command(pub, command[0], command[1])


if __name__ == "__main__":
    main()
