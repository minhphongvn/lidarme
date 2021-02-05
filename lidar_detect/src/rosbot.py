#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
"""


moveBindings = {
        # 'w':(1,0,0,0),
        # 'e':(1,0,0,-1),
        # 'a':(0,0,0,1),
        # 'd':(0,0,0,-1),
        # 'q':(1,0,0,1),
        # 'x':(-1,0,0,0),
        # 'c':(-1,0,0,1),
        # 'z':(-1,0,0,-1),
        'w':(1,0,0,0),
        'e':(1,0,0,-1),
        'k':(0,0,0,1),
        'l':(0,0,0,-1),
        'q':(1,0,0,1),
        's':(-1,0,0,0),
        'd':(-1,0,0,1),
        'a':(-1,0,0,-1),


        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def dangerCallback(msg):
    global distance
    distance = msg.data

distance = 999
key = "w"

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    rospy.Subscriber('/detect/lidar',Int32,dangerCallback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):

            key = "w"
            if distance < 150:
                print(distance)
                key = "k"

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            # elif key in speedBindings.keys():
            #     speed = speed * speedBindings[key][0]
            #     turn = turn * speedBindings[key][1]

            #     print(vels(speed,turn))
            #     if (status == 14):
            #         print(msg)
            #     status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)