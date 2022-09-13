#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import math
from sensor_msgs.msg import JointState


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.motors_num = 18
        self.joint_state = JointState()
        # joint_state.name = ["id_11", "id_21", "id_31", "id_12", "id_22", "id_32", "id_13", "id_23", "id_33",   
        #                 "id_41", "id_51", "id_61", "id_42", "id_52", "id_62", "id_43", "id_53", "id_63"] 
        self.joint_state.name = ["motor_1", "motor_2"] 
        self.joint_state.name = [0.0, -1.0] 
        self.joint_state.position = [0, 0, 0, 2, 2, 2, -1, -1, -1, 
                                0, 0, 0, 2, 2, 2, -1, -1, -1]
        self.joint_bias = [0, 0, 0, 2, 2, 2, -1, -1, -1, 
                    0, 0, 0, 2, 2, 2, -1, -1, -1]

        self.joint_state.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.joint_state.effort = [200, 200, 200, 200, 200, 200, 200, 200, 200, 
                                200, 200, 200, 200, 200, 200, 200, 200, 200]

        for i in range(self.motors_num):
            self.joint_state.velocity[i] = 0
            self.joint_state.effort[i] = 200
        o1 = 0.9
        o2 = 0.1


    def timer_callback(self):
        o1 = math.tanh(1.4*o1 + 0.3*o2 + 0.01)
        o2 = math.tanh(1.4*o2 - 0.3*o1 + 0.01)

        o1_com = o1*0.1
        o2_com = o2*0.1

        for i in range(self.motors_num):
            # if i%3 == 0:
            #     joint_state.position[i] = joint_bias[i] + o2_com
            # else:
            self.joint_state.position[i] = self.joint_bias[i] + o1_com

        self.publisher_.publish(self.joint_state)
        self.get_logger().info('Publishing: "%s"' % self.joint_state)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
