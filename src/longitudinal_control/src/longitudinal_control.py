#!/usr/bin/env python3
#
# Made by Jeonghun Kang

"""
longitudinal_control node

"""
# ROS Library
import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf import TransformListener

# Python Library
import numpy as np

# Subscribe Topic Type
from carmaker_msgs.msg import DynamicsInfo

# Publishe Topic Type
from carmaker_msgs.msg import Control_Signal

class LongitudinalControl(object):
    """
    Control With PID

    """

    def __init__(self):
        # ROS node
        rospy.init_node('longitudinal_control', anonymous=True)
        
        # Parameters
        self._hz = rospy.get_param('/hz')
        self._static_steering_deg = rospy.get_param('/static_steering')
        self._target_velocity_mps = rospy.get_param('/target_velocity')

        self._p_gain = rospy.get_param('/pid_control/p')
        self._i_gain = rospy.get_param('/pid_control/i')
        self._d_gain = rospy.get_param('/pid_control/d')
        
        # Input Variables
        self.i_vehicle_state = DynamicsInfo()
        
        # Output Variables
        self.o_control_command = Control_Signal()

        # Environment
        self._tf_listener = TransformListener()

        # Publisher
        self.p_control_command = rospy.Publisher(
            "carmaker/control_signal", Control_Signal, queue_size=10)
        # Subscriber
        self.s_vehicle_state = rospy.Subscriber(
            "carmaker/dynamic_info", DynamicsInfo, self.callback_vehicle_state)

    def destroy(self):
        """
        Destroy all objects
        """
        self.p_control_command.unregister()
        self.s_vehicle_state.unregister()
    
    def callback_vehicle_state(self, data):
        """
        Callback function for /vehicle_state
        """
        self.i_vehicle_state = data

    def publish(self):
        """
        Publish Control Command 
        """
        
        self.p_control_command.publish(self.o_control_command)

    def update_control_command(self,control_command):
        self.o_control_command.steerangle = self._static_steering_deg*(np.pi/180.0)
        if control_command > 1:
            self.o_control_command.gas = 1
            self.o_control_command.brake = 0
        elif control_command > 0:
            self.o_control_command.gas = control_command
            self.o_control_command.brake = 0
        elif control_command < -1:
            self.o_control_command.gas = 0
            self.o_control_command.brake = 1
        else:
            self.o_control_command.gas = 0
            self.o_control_command.brake = abs(control_command)

    def pid_control(self,vehicle_state):
        velocity_error = self._target_velocity_mps - vehicle_state.Car_vx
        
        return self._p_gain * velocity_error

    def run(self):
        """
        main loop
        """
        rate = rospy.Rate(self._hz)

        while not rospy.is_shutdown():
            # Get Current Inputs
            vehicle_state = self.i_vehicle_state
            
            # PID Control
            longitudinal_control_command = self.pid_control(vehicle_state)
            
            # Update Outputs
            self.update_control_command(longitudinal_control_command)

            # Publish 
            self.publish()

            rate.sleep()
        
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    """
    main function
    """
    longitudinal_control = LongitudinalControl()
    try:
        longitudinal_control.run()
    finally:
        if longitudinal_control is not None:
            longitudinal_control.destroy()


if __name__ == '__main__':
    main()