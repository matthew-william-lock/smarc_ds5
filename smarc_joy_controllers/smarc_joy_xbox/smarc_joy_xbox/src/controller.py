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

# Interface between Joy messages and the smarc_joy_controller for the Xbox controller

import rospy

from std_msgs.msg import Bool

from sensor_msgs.msg import Joy

from smarc_joy_msgs.msg import JoyButtons

import threading

class xbox_joy():

    def __init__(self):

        rospy.init_node('ds5_teleop', anonymous=True)
        rate = rospy.Rate(1) # 10hz

        self.button_pressed_flag = threading.Event()
        self.enable_teleop_pressed = False
        self.teleop_enabled = False

        self.teleop_enabled_pub = rospy.Publisher('ctrl/teleop/enable', Bool, queue_size=1)
        self.joy_btn_pub = rospy.Publisher('ctrl/joy_buttons', JoyButtons, queue_size=1)

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        rospy.loginfo("[XBOX CONTROLLER] Starting Xbox controller node")

        while not rospy.is_shutdown():
            self.send_teleop_enabled(self.teleop_enabled)
            rate.sleep()  

    # ================================================================================
    # Callbacks
    # ================================================================================

    def joy_callback(self, msg:Joy):
        """
        Callback function for the joystick subscriber
        """

        teleop_btn_pressed = msg.buttons[0] == 1

        if teleop_btn_pressed and not self.enable_teleop_pressed:
            self.teleop_enabled = not self.teleop_enabled
            self.enable_teleop_pressed = True
            rospy.loginfo("[XBOX CONTROLLER] Teleop enabled: {}".format(self.teleop_enabled))

        if not teleop_btn_pressed:
            self.enable_teleop_pressed = False
            
        joy_buttons_msg = JoyButtons()
        joy_buttons_msg.left_x = msg.axes[0]
        joy_buttons_msg.left_y = msg.axes[1]
        joy_buttons_msg.right_x = msg.axes[3]
        joy_buttons_msg.right_y = msg.axes[4]

        joy_buttons_msg.teleop_enable = self.teleop_enabled

        self.joy_btn_pub.publish(joy_buttons_msg)

        


    def send_teleop_enabled(self, teleop_enabled: bool):
        """
        Send a teleop enabled message to the core
        """
        
        teleop_enabled_msg = Bool()
        teleop_enabled_msg.data = teleop_enabled
        self.teleop_enabled_pub.publish(teleop_enabled_msg)

if __name__ == '__main__':
    try:
        xbox_joy()
    except rospy.ROSInterruptException:
        pass