#!/usr/bin/env python

import math
import numpy
import threading

from math import pi

import rospy
import tf

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers

class _LittleAckermannCtrlr(object):

    # [ ] - Build init.
    def __init__(self):

        rospy.init_node("ackermann_cotnroller")

        # Front Wheels
        (left_steer_link_name, left_steer_ctrlr_name, left_front_axle_ctrlr_name, self._left_front_inv_circ) = self._get_front_wheel_params("left")
        (right_steer_link_name, right_steer_ctrlr_name, right_front_axle_ctlrlr_name, self._right_front_inv_circ) self._get_front_wheel_params("right")

        list_ctrlrs = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        list_ctrlrs.wait_for_service()

        # [x] - Command timeout
        try:
            self._cmd_timeout = float(rospy.get_param("~cmd_timeout", self._DEF_CMD_TIMEOUT))
        except:
            rospy.logwarn("The specified command timeout value is invalid. ")
            self._cmd_timeout = self._DEF_CMD_TIMEOUT
    
        # [X] - Publish frequency
        try:
            pub_freq = float(rospy.get_param("~publishing_frequency", self._DEF_PUB_FREQ))

            if pub_freq <= 0.0:
                raise ValueError()
        except:
            rospy.logwarn("The specified publish frequency is invalid. ")
            pub_freq = self._DEF_PUB_FREQ
        
        self._sleep_timer = rospy.Rate(pub_freq)

        # Define the last time Ackermann driving command was received.
        self._last_cmd_time = rospy.get_time()

        # Define Ackermann cmd lock
        #  - This will control access to steer_ang, steer_ang_vel, speed, accel
        self._ackermann_cmd_lock = threading.Lock()
        self._steer_ang = 0.0
        self._steer_ang_vel = 0.0
        self._speed = 0.0
        self._accel = 0.0

        self._last_steer_ang = 0.0
        self._theta_left = 0.0
        self._theta_right = 0.0

        self._last_speed = 0.0
        self._last_accel_limit = 0.0

        self._left_front_ang_vel = 0.0
        self._right_front_ang_vel = 0.0
        self._left_rear_ang_vel = 0.0
        self._right_rear_ang_vel = 0.0

        # Listen to the TF made availble during launch
        tfl = tf.TransformListener()

        # Define the front left and right steering links 
        lsl_pos = self._get_link_postion(tfl, left_steer_link_name)
        rsl_pos = self._get_link_postion(tfl, right_steer_link_name)

        self._joint_dist_div_2 = numpy.linalg.norm(lsl_pos - rsl_pos) / 2

        # PICK UP HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   

    # [ ] - Build spin.
    def spin(self):
    
    # [ ] - Build callback.
    def little_ackermann_cmd_cb(self, ackermann_cmd):
    
    # [X] - Build front wheel params.
    def _get_front_wheel_params(self, side):

        prefix = "~" + side + "_front_wheel/"

        steer_link_name = rospy.get_param(prefix + "steering_link_name" + side + "steering_link")

        steer_ctrlr_name = rospy.get_param(prefix + "steering_controller_name", side + "_steering_contoller")

        axle_ctrlr_name, inv_circ = self._get_common_wheel_params(prefix)

        return steer_link_name, steer_ctrlr_name, axle_ctrlr_name, inv_circ
    
    # [ ] - Build rear wheel params.
    def _get_rear_wheel_params(self, side):

    # [ ] - Build common wheel params.
    def _get_common_wheel_params(self, prefix):

    # [ ] - Build link positions.
    def _get_link_postion(self, tfl, link):

    # [ ] - Build control steering.
    def _ctrl_steering(self, steer_ang, steer_ang_vel_limit, delta_t):

    # [ ] - Build control axles.
    def _ctrl_axles(self, speed, accel_limit, delta_t, steer_ang_changed, center_y):

# End of LittleAckermannCtrlr Class

# [x] - Wait for the specified controller to in the running state.
def _wait_for_ctrlr(list_ctrlrs, ctrlr_name):

    while True:
        response = list_ctrlrs()
        
        for ctrlr in response.controller:
            if ctrlr.name == ctrlr_name:
                if ctrlr.state == "running":
                    return
                rospy.sleep(0.1)
                break

# [x] - Create axle command publisher.
def _create_axle_cmd_pub(list_ctrlrs, axle_ctrlr_name):

    if not axle_ctrlr_name:
        return None
    
    return _create_axle_cmd_pub(list_ctrlrs, axle_ctrlr_name)

# [x] - Create a command publisher.
def _create_cmd_pub(list_ctrlrs, ctrlr_name):

    _wait_for_ctrlr(list_ctrlrs, ctrlr_name)

    return rospy.Publisher(ctrlr_name + "/command", Float64, queue_size=1)

# [x] - Return the desired steering angle for a front wheel.
def _get_steer_ang(phi):

    if phi >= 0.0:
        return (pi / 2) - phi

    return (-pi / 2) - phi

# main
if __name__ == "__main__":
    ctrlr = _LittleAckermannCtrlr()
    ctrlr.spin()