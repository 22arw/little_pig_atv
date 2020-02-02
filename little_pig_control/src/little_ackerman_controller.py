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
    
    # [ ] - Build spin.
    def spin(self):
    
    # [ ] - Build callback.
    def little_ackermann_cmd_cb(self, ackermann_cmd):
    
    # [ ] - Build front wheel params.
    def _get_front_wheel_params(self, side):
    
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