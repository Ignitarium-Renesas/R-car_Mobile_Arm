#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

TwistMsg = Twist

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (0, 0, 0, -1),
    'j': (0, 1, 0, 0),
    'l': (0, -1, 0, 0),
    'u': (0, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, -1),
    'm': (-1, 0, 0, 1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1),
    'x': (0.9, 1),
    'e': (1, 1.1),
    'c': (1, 0.9),
}


class PublishThread(threading.Thread):
    def __init__(self, node, rate):
        super(PublishThread, self).__init__()
        self.publisher = node.create_publisher(TwistMsg, 'cmd_vel', 10)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.timeout = 1.0 / rate if rate != 0.0 else None
        self.start()

    def update(self, x, y, z, th, speed, turn):
        with self.condition:
            self.x = x
            self.y = y
            self.z = z
            self.th = th
            self.speed = speed
            self.turn = turn
            self.condition.notify()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()
        while not self.done:
            with self.condition:
                self.condition.wait(self.timeout)
                twist_msg.linear.x = self.x * self.speed
                twist_msg.linear.y = self.y * self.speed
                twist_msg.linear.z = self.z * self.speed
                twist_msg.angular.z = self.th * self.turn
            self.publisher.publish(twist_msg)


def get_key(settings, timeout):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_terminal_settings():
    return None if sys.platform == 'win32' else termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    if sys.platform != 'win32':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return f"currently:\tspeed {speed}\tturn {turn}"


def main():
    settings = save_terminal_settings()

    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')

    speed = node.declare_parameter("speed", 0.25).value
    turn = node.declare_parameter("turn", 0.5).value
    speed_limit = node.declare_parameter("speed_limit", 1.0).value
    turn_limit = node.declare_parameter("turn_limit", 1.0).value
    repeat = node.declare_parameter("repeat_rate", 0.0).value
    key_timeout = node.declare_parameter("key_timeout", 0.5).value

    pub_thread = PublishThread(node, repeat)

    x = y = z = th = 0
    status = 0

    try:
        pub_thread.update(x, y, z, th, speed, turn)
        print(msg)
        print(vels(speed, turn))
        while True:
            key = get_key(settings, key_timeout)
            if key in moveBindings:
                x, y, z, th = moveBindings[key]
            elif key in speedBindings:
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                print(vels(speed, turn))
            else:
                x = y = z = th = 0
                if key == '\x03':
                    break
            pub_thread.update(x, y, z, th, speed, turn)
    except Exception as e:
        print(e)
    finally:
        pub_thread.stop()
        restore_terminal_settings(settings)


if __name__ == "__main__":
    main()
