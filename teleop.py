#!/usr/bin/python

import rospy
import sys, select, termios, tty
import geometry_msgs.msg
import time




MANIPULATOR_STEPS = 500
SPEED_STEPS = 5


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def parse_buttons(speed, key):
    global force,torque
    msgForce1=geometry_msgs.msg.Wrench()
    if key == 'w':
	msgForce1.force.x=force
        pub_manipulator.publish(msgForce1)
    if key == 's':
	msgForce1.torque.z=0
	msgForce1.force.x=0
        pub_manipulator.publish(msgForce1)
    if key == 'a':
	msgForce1.torque.z=torque
        pub_manipulator.publish(msgForce1)
    if key == 'd':
	msgForce1.torque.z=-torque
        pub_manipulator.publish(msgForce1)
    if key == 'e':
	msgForce1.torque.z=0
        pub_manipulator.publish(msgForce1)
    if key == 'r':
	force+=0.1
    if key == 'f':
	force-=0.1
    if key == 't':
	torque+=0.05
    if key == 'g':
	torque-=0.05
    return 0


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    speed = 0
    force=600
    torque=300
    
    current_longt=0    

    rospy.init_node('stepper_teleop')

    pub_manipulator = rospy.Publisher("/Maran/thruster_manager/input", geometry_msgs.msg.Wrench, queue_size=1)
    print("kik")
    longtitude_sub=rospy.Subscriber("/Maran/gps/longtitude",data,current_longt,queue_size = 10)
    print(current_longt)
    
    try:
        while(1):
            key = getKey()
            speed = parse_buttons(speed, key)
            if key == '\x03':
                raise KeyboardInterrupt()
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
