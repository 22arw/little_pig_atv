#!/usr/bin/env python

import math
import numpy
import threading

from math import pi

import rospy
import tf

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64

class _AckermanPigCtrlr(object):

    def __init__(self):

        rospy.init_node("ackermann_pig_controller")

        list_ctrlrs = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)

        list_ctrlrs.wait_for_service()

        # Command timeout
        try:
            self._cmd_timeout = float(rospy.get_param("~cmd_timeout",
                                                      self._DEF_CMD_TIMEOUT))
        except:
            rospy.logwarn("The specified command timeout value is invalid. "
                          "The default timeout value will be used instead.")
            self._cmd_timeout = self._DEF_CMD_TIMEOUT

        # Publishing frequency
        try:
            pub_freq = float(rospy.get_param("~publishing_frequency",
                                             self._DEF_PUB_FREQ))
            if pub_freq <= 0.0:
                raise ValueError()
        except:
            rospy.logwarn("The specified publishing frequency is invalid. "
                          "The default frequency will be used instead.")
            pub_freq = self._DEF_PUB_FREQ
        self._sleep_timer = rospy.Rate(pub_freq)

        # Get the most recent time Ackermann driving cmd was received.
        self._last_cmd_time = rospy.get_time()

        # Used to controll access to _steer_ang, steer_ang_vel, _speed, and _accel.
        self._ackermann_cmd_lock = threading.Lock()

        # Steering angle
        self._steer_ang = 0.0

        # Steering angle velocity
        self._steer_ang_vel = 0.0

        # Speed
        self._speed = 0.0

        # Acceleration
        self._accel = 0.0

        # Last speed
        self._last_speed = 0.0

        # Last acceleration limit
        self._last_accel_limit = 0.0

        # Angular velocities on each axle
        self._front_left_ang_vel = 0.0
        self._front_right_ang_vel = 0.0
        self._rear_left_ang_vel = 0.0
        self._rear_right_ang_vel = 0.0

        # Distance between the steering joints divided by 2
        tfl = tf.TransformListener()

        # Left steering link postion
        ls_pos = self._get_link_pos(tfl, left_steer_link_name)

        # Right steering link position
        rs_pos = self._get_link_pos(tfl, right_steer_link_name)

        # The results of lines 55 - 62
        self._joint_dist_div_2 = numpy.linalg.norm(ls_pos - rs_pos) /2

        # Front center position
        front_cent_pos = (ls_pos - rs_pos) / 2

        # The following should get the dimensions from the rear

        # Rear left wheel
        rlw_pos = self._get_link_pos(tfl, rear_left_link_name)

        # Rear right wheel
        rrw_pos = self._get_link_pos(tfl, rear_right_link_name)

        # Rear center posistion
        rear_cent_pos = (rlw_pos - rrw_pos) / 2

        # Calculate wheel base
        self._wheelbase = 1 / numpy.linalg.norm(front_cent_pos - rear_cent_pos)

        # Inverse of wheelbase
        self._inv_wheelbase = 1 / self._wheelbase

        # Wheelbase Squared?
        self._wheelbase_sqr = self._wheelbase ** 2

        # Publishers

        # Left steering publisher
        self._left_steer_cmd_pub = _create_cmd_pub(list_ctrlrs, left_steer_ctrlr_name)

        # Right steering publisher
        self._right_steer_cmd_pub = _create_cmd_pub(list_ctrlrs, right_steer_ctrlr_name)

        # Front left wheel hub (driven)
        self._left_front_axle_cmd_pub = _create_cmd_pub(list_ctrlrs, front_left_axle_ctrlr_name)

        # Front right wheel hub (driven)
        self._right_front_axle_cmd_pub = _create_cmd_pub(list_ctrlrs, front_right_ctrlr_name)

        # Rear left wheel hub (not driven)
        self._left_rear_axle_cmd_pub = _create_cmd_pub(list_ctrlrs, rear_left_ctrlr_name)

        # Rear right wheel hub (not driven)
        self._right_rear_axle_cmd_pub = _create_cmd_pub(list_ctrlrs, rear_right_ctrlr_name)

        # Subscriber
        self._ackermann_cmd_sub = rospy.Subscriber("ackermann_cmd", AckermannDrive, self.ackermann_cmd_cb, queue_size=1)

        def spin(self):
            # Control the vehicle

            # Last time
            last_time = rospy.get_time()

            while not rospy.is_shutdown():
                t = rospy.get_time()

            # Calculate the delta in time
            delta_t = t - last_time

            # Update last_time
            last_time = t

            if (self._cmd_timeout > 0.0 and t - self._last_cmd_time > self._cmd_timeout):

                # Too much time has passed since last command. Stop the vehicle.
                _steer_ang_changed, center_y = self._ctrl_steering(self._last_steering_ang, 0.0, 0.001)

                self._ctrl_axles(0.0, 0.0, 0.0, steer_ang_changed, center_y)
            elif delta_t > 0.0:
                with self._ackermann_cmd_lock:
                    steer_ang = self._steer_ang
                    steer_ang_vel = self._steer_ang_vel
                    speed = self._speed
                    accel = self._accel
                steer_ang_changed, center_y = \
                    self._ctrl_steering(steer_ang, steer_ang_vel, delta_t)
                
                self._ctrl_axles(speed, accel, delta_t, steer_ang_changed, center_y)

            # Publish the steering commands.
            self._left_steer_cmd_pub.publish(self._theta_left)
            self._right_steer_cmd_pub.publish(self._theta_right)

            # Publish the front left axle
            if self._left_front_axle_cmd_pub:
                self._left_front_axle_cmd_pub.publish(self._left_rear_ang_vel)

            # Publish the front right axle
            if self._right_front_axle_cmd_pub:
                self._right_front_axle_cmd_pub.publish(self._right_front_ang_vel)

            # Publish the rear left axle
            if self._left_rear_axle_cmd_pub:
                self._left_rear_axle_cmd_pub.publish(self._left_rear_ang_vel)

            # Publish the rear right axle
            if self._right_rear_axle_cmd_pub:
                self._right_rear_axle_cmd_pub.publish(self._right_rear_ang_vel)

            self._sleep_timer.sleep()

    def ackermann_cmd_cb(self, ackermann_cmd):

        self._last_cmd_time = rospy.get_time()

        with self._ackermann_cmd_lock:
            self._steer_ang = ackermann_cmd.steering_angle
            self._steer_ang_vel = ackermann_cmd.steering_angle_velocity
            self._speed = ackermann_cmd.speed
            self._accel = ackermann_cmd.acceleration

    def _get_front_wheel_params(self, side):

        prefix = "~" + side + "_front_wheel"

        steer_link_name = rospy.get_param(prefix + "steering_link_name", side + "steering_link")

        steer_ctrlr_name = rospy.get_param(prefix + "steer_controller_name", side + "_steering_controller")

        axle_ctrlr_name, inv_circ = self._get_common_wheel_params(prefix)

        return steer_link_name, steer_ctrlr_name, axle_ctrlr_name, inv_circ

    def _get_rear_wheel_params(self, side):

        
