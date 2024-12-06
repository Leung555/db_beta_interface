#!/usr/bin/env python3
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

import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import math
from sensor_msgs.msg import JointState


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # initiate publisher
        self.publisher_ = self.create_publisher(JointState, 'motor_command', 10)
        timer_period = 0.03  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # number of motors
        self.num_motors = 18

        # Initialize Joint state
        self.joint_state = JointState()
        # joint_state.name = ["id_11", "id_21", "id_31", "id_12", "id_22", "id_32", "id_13", "id_23", "id_33",   
        #                 "id_41", "id_51", "id_61", "id_42", "id_52", "id_62", "id_43", "id_53", "id_63"] 
        # self.joint_state.name = ["motor_1", "motor_2"] 
        # self.joint_state.position = [0, 0]
        self.joint_bias = [0.0] * self.num_motors

        self.joint_bias = []
        BC_bias = 0
        CF_bias = -0.785 #(rad) = -45 deg
        FT_bias = +0.785 #(rad) = -45 deg
        for i in range(self.num_motors//3):
            self.joint_bias.append(BC_bias)
            self.joint_bias.append(CF_bias)
            self.joint_bias.append(FT_bias)
        print('self.joint_bias: ', self.joint_bias)

        # self.joint_bias = [0.0]
        # self.joint_state.velocity = [0, 0]
        # self.joint_state.effort = [200, 200]

        # # initialize default variables
        # for i in range(self.motors_num):
        #     self.joint_state.velocity[i] = 0
        #     self.joint_state.effort[i] = 200

        # CPG
        MI = 0.1
        self.w11, self.w22 = 1.4, 1.4
        self.w12 =  0.18 + MI
        self.w21 = -0.18 - MI
        
        self.o1 = 0.01
        self.o2 = 0.01

        # Leg config
        self.BC_joints = [0, 3, 6, 9, 12, 15]
        self.CF_joints = [1, 4, 7, 10, 13, 16]
        self.FT_joints = [2, 5, 8, 11, 14, 17]
        print('FT_joints: ', self.FT_joints)

        # program counter
        self.counter = 0
        self.ep_length = 200
        self.actuate = True

    def timer_callback(self):
        # Reinitialize the JointState message based on the number of motors
        self.joint_state.name = [f"motor_{i+1}" for i in range(self.num_motors)]
        self.joint_state.position = [0.0] * self.num_motors
        self.joint_state.velocity = [0.0] * self.num_motors
        self.joint_state.effort = [0.0] * self.num_motors

        # CPG
        self.o1 = math.tanh(self.w11*self.o1 + self.w12*self.o2)
        self.o2 = math.tanh(self.w22*self.o2 + self.w21*self.o1)

        o1_com = abs(self.o1*0.6)
        o2_com = abs(self.o2*0.6)

        # gait scheduler
        if self.o1 > 0.2:
            phase = 1
        else:
            phase = 0
        
        # motor mapping
        if self.actuate:
            if phase == 1:
                for i in self.CF_joints:
                    if i in [1, 4, 13]:
                        self.joint_state.position[i] = self.joint_bias[i] + o1_com
                    elif i in [7, 10, 16]:
                        self.joint_state.position[i] = self.joint_bias[i]
                for i in self.FT_joints:
                    if i in [2, 5, 14]:
                        self.joint_state.position[i] = self.joint_bias[i] + o1_com
                    elif i in [8, 11, 17]:
                        self.joint_state.position[i] = self.joint_bias[i]
            if phase == 0:
                for i in self.CF_joints:
                    if i in [1, 4, 13]:
                        self.joint_state.position[i] = self.joint_bias[i] 
                    elif i in [7, 10, 16]:
                        self.joint_state.position[i] = self.joint_bias[i] + o1_com
                for i in self.FT_joints:
                    if i in [2, 5, 14]:
                        self.joint_state.position[i] = self.joint_bias[i]
                    elif i in [8, 11, 17]:
                        self.joint_state.position[i] = self.joint_bias[i] + o1_com

        # Testing simple motor control
        # for i in range(self.num_motors):
        #     self.joint_state.position[i] = self.joint_bias[i] + o1_com

        # Update the header timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()


        self.publisher_.publish(self.joint_state)
        # minimal verbose
        # self.get_logger().info('Publishing: "%s"' % self.get_clock().now().to_msg())
        # self.get_logger().info('Publishing: "%s"' % self.joint_state)

        # Check if the counter exceeds the threshold
        if self.counter > self.ep_length:
            self.get_logger().info(f'Counter exceeded the threshold of {self.ep_length}. Shutting down...')
            # Destroy the node before shutting down
            # self.destroy_node()
            rclpy.shutdown()  # Gracefully stop the program
        print('counter: ', self.counter)
        self.counter += 1


def main(args=None):
    # for i in range(3, 0, -1):
    #     time.sleep(1)
    #     print("Program running in ", i)
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
