#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

from read_encoders.srv import CalibrateWheels, CalibrateWheelsResponse

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

        # Services
        self.calibrate_srv = rospy.Service('~calibrate_wheels', CalibrateWheels, self.handle_calibrate_wheels)

        self.log("Initialized")

        self.left_init = None
        self.right_init = None

        self.left_ticks = 0
        self.right_ticks = 0

        self.left_direction = FORWARD
        self.right_direction = FORWARD


    def handle_calibrate_wheels(self, req):
        # TODO theta needs to be converted to radians
        Rr, Rl = self.calibrateLeastSquares(req.x, req.y, np.deg2rad(req.theta))
        response = 'Rr = {}   Rl = {}'.format(Rr, Rl)
        print(response)
        return CalibrateWheelsResponse(response)

    def calibrateLeastSquares(self,x,y,theta):
        ''' Assume robot starts at (x=0, y=0, theta=0)
            and drives in a straight line for a short period of time 
            to a measured position of (x, y, theta)

            We have the following relationships:
            x = d*cos(theta)
            y= d*sin(theta)
            theta = (dr - dl) / 2L
            where d = (dr + dl) / 2
            and each dr and dl = 2pi(N_{r,l}/Nrev)*R_{r,l} = C_{r,l}*R_{r,l}
            Rr and Rl enter the equations linearly and thus can be
            estimated using least squares

            AR = b
            b = [x y theta]'
            R = [Rr Rl]'
            A = [[Cr*cos(theta)/2   Cl*cos(theta)/2]
                 [Cr*sin(theta)/2   Cl*sin(theta)/2]
                 [Cr/2L             -Cl/2L         ]]

            *Note that the theta in cos and sin are approximated to be theta/2
        '''
        L = self._baseline
        avg_theta = theta / 2.0

        Cr = 2 * np.pi * (self.right_ticks / N_REV)
        Cl = 2 * np.pi * (self.left_ticks / N_REV)

        b = np.array([x, y, theta]).reshape((3,1))

        A = np.array([[Cr*np.cos(avg_theta)/2.0, Cl*np.cos(avg_theta)/2.0],
                      [Cr*np.sin(avg_theta)/2.0, Cl*np.sin(avg_theta)/2.0],
                      [Cr/(2.0*L), -Cl/(2.0*L)]])

        R = np.dot(np.linalg.pinv(A), b).squeeze()

        return R[0], R[1]

    def cb_encoder_data(self, wheel, msg):
        if wheel == 'left':
            if self.left_init == None:
                self.left_init = msg.data
            else:
                self.left_ticks = msg.data - self.left_init
                self.pub_integrated_distance_left.publish(self._C * (self.left_ticks))

        elif wheel == 'right':
            if self.right_init == None:
                self.right_init = msg.data
            else:
                self.right_ticks = msg.data - self.right_init
                self.pub_integrated_distance_right.publish(self._C * (self.right_ticks))
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
    