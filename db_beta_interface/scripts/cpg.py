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
        self.joint_state = JointState()
        self.num_motors = 2
        self.joint_state.name = [f"motor_{i+1}" for i in range(self.num_motors)]
        self.joint_bias = [0.0] * self.num_motors
        

        self.counter      = 0
        self.cycle_time   = 20
        self.ending_step  = 1000

        self.stay_time = 1
        self.joint_state.name = [f"motor_{i+1}" for i in range(self.num_motors)]
        
        # Define positions
        # self.position = [1, 0]
        # self.resting_position = [0, 0]
        self.motor_1 = [0.2, 0  ]
        self.motor_2 = [0  , 0.2]
        self.command_index = 0


    def timer_callback(self):

        # Reinitialize the JointState message based on the number of motors
        self.joint_state.position = [0.0] * self.num_motors
        self.joint_state.velocity = [0.0] * self.num_motors
        self.joint_state.effort = [0.0] * self.num_motors

        # Update the header timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        self.joint_state.position[0] = self.motor_1[self.command_index]
        self.joint_state.position[1] = self.motor_2[self.command_index]
        print('self.motor_1[self.command_index]: ', self.motor_1[self.command_index])

            
        # if self.counter < self.cycle_time//2:
        #     for i in range(self.num_motors):
        #         self.joint_state.position[i] = self.position[i]
        # elif self.counter > self.cycle_time//2:
        #     for i in range(self.num_motors):
        #         self.joint_state.position[i] = self.resting_position[i]
        
        if self.counter > self.cycle_time:
            self.counter = 0
            self.command_index += 1
            print('self.command_index: ', self.command_index)
            if self.command_index > len(self.motor_1)-1:
                self.command_index = 0

        self.publisher_.publish(self.joint_state)

        # minimal verbose
        # self.get_logger().info('Publishing: "%s"' % self.get_clock().now().to_msg())
        # self.get_logger().info('Publishing: "%s"' % self.joint_state)

        # Check if the counter exceeds the threshold
        if self.counter > self.ending_step:
            self.get_logger().info(f'Counter exceeded the threshold of {self.ep_length}. Shutting down...')
            # Destroy the node before shutting down
            # self.destroy_node()
            rclpy.shutdown()  # Gracefully stop the program
        # print('counter: ', self.counter)
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
