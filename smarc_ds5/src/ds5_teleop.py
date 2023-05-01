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

#!/usr/bin/env python3

# Create a ROS node that republishes a joy message


import rospy

from sensor_msgs.msg import JoyFeedbackArray, Joy
from smarc_msgs.msg import ThrusterRPM
from sam_msgs.msg import ThrusterAngles

class ds5_teleop():

    def joy_callback(self, msg:Joy):
        """
        Callback function for the joystick subscriber
        """

        RPM_MAX = 1500
        RAD_MAX = 0.1

        rps_cmd = msg.axes[1] * RPM_MAX
        x_cmd = msg.axes[2] * RAD_MAX
        y_cmd = msg.axes[3] * RAD_MAX

        rpm1_msg = ThrusterRPM()
        rpm2_msg = ThrusterRPM()
        rpm1_msg.rpm = rps_cmd
        rpm2_msg.rpm = rps_cmd

        angle_msg = ThrusterAngles()
        angle_msg.thruster_vertical_radians = y_cmd
        angle_msg.thruster_horizontal_radians = x_cmd

        self.rpm1_pub.publish(rpm1_msg)
        self.rpm2_pub.publish(rpm2_msg)
        self.angle_pub.publish(angle_msg)
        
        pass


    def __init__(self):

        # pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('ds5_teleop', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        # Thruster publisher
        self.rpm1_pub = rospy.Publisher('core/thruster1_cmd', ThrusterRPM, queue_size=1000)
        self.rpm2_pub = rospy.Publisher('core/thruster2_cmd', ThrusterRPM, queue_size=1000)

        # Angle publisher
        self.angle_pub = rospy.Publisher('core/thrust_vector_cmd', ThrusterAngles, queue_size=1000) 


        # Joy subscriber
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        ds5_teleop()
    except rospy.ROSInterruptException:
        pass