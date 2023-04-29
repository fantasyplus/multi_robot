#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty

msg = """
a : change robot queue style
s : start to move
q/z : increase/decrease max speeds by 10%
space key : force stop
c : quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %f\tturn %f " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('ares_teleop')
    vel_pub = rospy.Publisher('ares1/cmd_vel',  Twist,   queue_size=1)
    global_vel_pub = rospy.Publisher('cmd_vel',  Twist,   queue_size=1)
    flag_pub = rospy.Publisher('flag',  String,   queue_size=1)

    index = 1
    speed_x = 0
    yaw = 0
    try:
        print msg
        while(1):
            key = getKey()
            # 速度修改键
            if key == 'q':
                speed_x = speed_x * 1.1  # 线速度增加0.1倍
                print vels(speed_x,yaw)
            elif key == 'z':
                speed_x = speed_x * 0.9  # 线速度减少0.1倍
                print vels(speed_x,yaw)

            # 停止键
            if key == ' ':
                speed_x=0
                yaw=0
                print "stop"
            # change queue
            elif key == 'a':
                list = ["line", "triangle"]
                output = list[index]
                index += 1
                if index == len(list):
                    index = 0

                print "Change queue to %s" % output
                flag_pub.publish(output)
             # start to move
            elif key == 's':
                speed_x = 1.5
                print "start to move"
            elif key == 'c':
                print "exit"
                break

            # 创建并发布twist消息
            twist = Twist()
            twist.linear.x = speed_x
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0 
            twist.angular.y = 0 
            twist.angular.z = yaw

            vel_pub.publish(twist)
            global_vel_pub.publish(twist)

    except:
        print("error")

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        vel_pub.publish(twist)
        global_vel_pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
