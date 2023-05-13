#!/usr/bin/env python3

"""
MIT License

Copyright (c) 2023 Matthew Lock

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

# Create a ROS node that republishes a joy message


import rospy

from sensor_msgs.msg import JoyFeedbackArray, Joy
from smarc_msgs.msg import ThrusterRPM
from sam_msgs.msg import ThrusterAngles

from ds5_msgs.msg import SetColour
from ds5_msgs.msg import SetMotor

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from smarc_joy_msgs.msg import JoyButtons

import threading
import time
import math

class teleop():
    
    # ================================================================================
    # Callback Functions
    # ================================================================================

    def joy_btns_callback(self, msg:JoyButtons):
        """
        Callback function for the joystick subscriber
        """

        if self.teleop_enabled:

            RPM_MAX = 1500
            RAD_MAX = 0.1
            RAD_STEPS = 5
            RAD_STEP_SIZE = RAD_MAX / RAD_STEPS

            rpm_cmd = int(msg.left_y* RPM_MAX)
            x_cmd = msg.right_x * RAD_MAX
            y_cmd = msg.right_y * RAD_MAX

            # Round rpm_cmd to nearest 100
            rpm_cmd = int(round(rpm_cmd, -2))

            # Round x_cmd and y_cmd to nearest value on range RAD_MIN to RAD_MAX with RAD_STEPS steps
            x_steps = round(x_cmd / RAD_STEP_SIZE)
            x_cmd = x_steps * RAD_STEP_SIZE
            y_steps = round(y_cmd / RAD_STEP_SIZE)
            y_cmd = y_steps * RAD_STEP_SIZE

            x_cmd = math.degrees(x_cmd)
            y_cmd = math.degrees(y_cmd)

            ctrl_msg = Twist()
            ctrl_msg.linear.x = rpm_cmd
            ctrl_msg.angular.z = x_cmd
            ctrl_msg.angular.y = y_cmd

            self.rpm_joystick_pub.publish(ctrl_msg)
            self.vector_deg_joystick_pub.publish(ctrl_msg)

            # rospy.loginfo("RPM: %d, X: %f, Y: %f", rpm_cmd, x_cmd, y_cmd)
            
    def teleop_enabled_callback(self, msg: Bool):
        """
        Callback function for the teleop enabled subscriber
        """
        self.teleop_enabled = msg.data

    # ================================================================================
    # Node Init
    # ================================================================================

    def __init__(self):
        
        # Threading flags
        self.button_pressed_flag = threading.Event()

        # pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('ds5_teleop', anonymous=True)
        rate = rospy.Rate(1) # 10hz

        # Publishers
        self.rpm_joystick_pub = rospy.Publisher('ctrl/rpm_joystick', Twist, queue_size=1)
        self.vector_deg_joystick_pub = rospy.Publisher('ctrl/vector_deg_joystick', Twist, queue_size=1)

        # Subscribers
        self.joy_btn_sub = rospy.Subscriber('ctrl/joy_buttons', JoyButtons, self.joy_btns_callback)
        self.teleop_enabled_sub = rospy.Subscriber('ctrl/teleop/enable', Bool, self.teleop_enabled_callback)

        # States
        self.teleop_enabled = False

        while not rospy.is_shutdown():
            rate.sleep()        

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass