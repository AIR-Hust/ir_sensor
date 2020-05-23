#!/usr/bin/env python

import rospy
from ros_arduino_python.arduino_driver import Arduino
from ir_arduino_drivers_pcl import IR_sensor_arr

import os, time
import thread

class IR_Array_ROS():
    def __init__(self):
        rospy.init_node("IR_array", log_level=rospy.INFO)

        rospy.on_shutdown(self.shutdown)

        # load params:
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        self.rate = int(rospy.get_param("~rate", 150))
        self.sensor_rate = int(rospy.get_param("~sensor_rate", 100))
        r = rospy.Rate(self.rate)

        now = rospy.Time.now()
        self.t_delta_sensors = rospy.Duration(1.0 / self.sensor_rate)
        self.t_next_sensors = now + self.t_delta_sensors

        self.controller = Arduino(self.port, self.baud, self.timeout)
        self.controller.connect()

        rospy.loginfo("Connected to Arduino on port: {} at {} baud".format(self.port, self.baud))

        mutex = thread.allocate_lock()
        self.sensorPins = rospy.get_param("~pins")
        self.sensorAngles = rospy.get_param("~angles")

        rospy.loginfo('sensor Pins: {}'.format(self.sensorPins))
        rospy.loginfo('sensor Angles: {}'.format(self.sensorAngles))

        sensor_arr = IR_sensor_arr(self.controller, self.sensorPins, self.sensorAngles, self.sensor_rate, self.base_frame)
        while not rospy.is_shutdown():
            mutex.acquire()
            sensor_arr.poll()
            mutex.release()

            r.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down Arduino Node...")

# class Ir_filter(object):
#     def __init__(self):
#         self.result_rate = int(rospy.get_param("~result_rate", 10))
#         print(self.result_rate)

if __name__ == '__main__':
    myIRarray = IR_Array_ROS()
    # ir_filter = Ir_filter()
