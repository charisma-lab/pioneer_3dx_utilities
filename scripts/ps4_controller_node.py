#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy



ROTATION_AXIS_IDX= 0
LINEAR_AXIS_IDX = 1
STOP_BUTTON = 0

LIN_MAX = 2.0
ANG_MAX = 0.5


vel = Twist()

def sendRobotTwist(dx, dw, stop=True):
    if(not stop):
        vel.angular.z = dw
        vel.linear.x = dx
    else:
        vel.angular.z = 0
        vel.linear.x = 0

    pub.publish(vel)

def joyCB(joy_msg):
    if(joy_msg.buttons[STOP_BUTTON]):
        # stop everything
        sendRobotTwist(0,0,True)
        return

    joy_x = joy_msg.axes[LINEAR_AXIS_IDX]
    joy_w = joy_msg.axes[ROTATION_AXIS_IDX]

    sendRobotTwist(joy_x * LIN_MAX, joy_w * ANG_MAX, False)
    
    

pub = rospy.Publisher('/rosaria/cmd_vel', Twist, queue_size=10)
sub = rospy.Subscriber('/joy', Joy, joyCB)        
    
    
if __name__ == '__main__':
    rospy.init_node('Robot_CMDR', anonymous=True)
    rospy.spin()
    
    
    
