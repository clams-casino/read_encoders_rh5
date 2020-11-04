#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

FORWARD = 1
REVERSE = 0
N_REV = 135


class ReadEncodersNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(ReadEncodersNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._baseline = rospy.get_param(f'/{self.veh_name}/kinematics_node/baseline')
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius')
        self._C = 2.0*np.pi*self._radius / N_REV     # Precompute this multiplier


        # Subscribers
        self.sub_encoder_ticks_left = rospy.Subscriber('left_wheel_encoder_node/tick', 
                                                        WheelEncoderStamped,
                                                        lambda msg: self.cb_encoder_data('left', msg))

        self.sub_encoder_ticks_right = rospy.Subscriber('right_wheel_encoder_node/tick', 
                                                        WheelEncoderStamped, 
                                                        lambda msg: self.cb_encoder_data('right', msg))

        self.sub_executed_commands = rospy.Subscriber('wheels_driver_node/wheels_cmd_executed',
                                                      WheelsCmdStamped, self.cb_executed_commands)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher('~left_distance', Float32, queue_size=10)
        self.pub_integrated_distance_right = rospy.Publisher('~right_distance', Float32, queue_size=10)

        self.log("Initialized")

        self.left_init = None
        self.right_init = None

        self.left_direction = FORWARD
        self.right_direction = FORWARD

        self.left_distance = 0
        self.right_distance = 0

    def cb_encoder_data(self, wheel, msg):
        if wheel == 'left':
            if self.left_init == None:
                self.left_init = msg.data
            else:
                self.left_distance = self._C * (msg.data - self.left_init)
                self.pub_integrated_distance_left.publish(self.left_distance)

        elif wheel == 'right':
            if self.right_init == None:
                self.right_init = msg.data
            else:
                self.right_distance = self._C * (msg.data - self.right_init)
                self.pub_integrated_distance_right.publish(self.right_distance)

        else:
            rospy.logwarn("Invalid wheel. Either left or right")

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        if msg.vel_left >= 0:
            self.left_direction = FORWARD
        else:
            self.left_direction = REVERSE

        if msg.vel_right >= 0:
            self.right_direction = FORWARD
        else:
            self.right_direction = REVERSE

if __name__ == '__main__':
    node = ReadEncodersNode(node_name='read_encoders_node')
    rospy.loginfo("read_encoders_node is up and running...")
    # Keep it spinning to keep the node alive
    rospy.spin()
    