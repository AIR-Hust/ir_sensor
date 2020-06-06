#!/usr/bin/env python

"""
IR sensor for arduino IR sensor reader
Created for EAI robot expand by nvHuy
Feb 23, 2020
"""

import roslib; roslib.load_manifest("ros_arduino_python")
import rospy
from ir_sensor.msg import IR_Array_msg

from math import pi as PI, degrees, radians, sin, cos
from sensor_msgs.msg import Range
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

BASE_RADIUS = 0.145  #cm

class IR_sensor_arr(object):
    def __init__(self, controller, pins, angles, rate, frame_id):
        self.controler = controller
        self.pins = pins
        self.angles = angles
        self.rate = rate
        self.frame_id = frame_id

        self.values = [0.8]*len(pins)

        self.msg = IR_Array_msg()
        self.msg.header.frame_id = self.frame_id
        self.msg.min_range = 0.10
        self.msg.max_range = 0.80
        self.msg.angles = self.angles

        self.pub = rospy.Publisher("IR_sensor_arr", IR_Array_msg, queue_size=10)

        #ir pcl:
        self.ir_pcl_pub = rospy.Publisher("/ir_cloudpoint", PointCloud2, queue_size=5)
        #-- params
        self.ir_sensor_angles = rospy.get_param("~angles")
        self.ir_sensor_angles_rad = [x*PI/180.0 for x in self.ir_sensor_angles]
        rospy.loginfo("IR sensor angles: {}".format(self._list(self.ir_sensor_angles_rad)))
        self.ir_height = 0.100
        self.ir_maxval = 0.80 #cm
        self.ir_cloud = [[0.1,0.1,0.1] for j in range(10)]
        # Khoi tao ir_cloud
        for i in range(len(self.ir_sensor_angles)):
            self.ir_cloud[i][0] = (BASE_RADIUS + self.ir_maxval) \
                                    * cos(self.ir_sensor_angles_rad[i])
            self.ir_cloud[i][1] = (BASE_RADIUS + self.ir_maxval) \
                                    * sin(self.ir_sensor_angles_rad[i])
            self.ir_cloud[i][2] = self.ir_height

        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            try:
                idx = 0
                for pin in self.pins:
                    if (idx<=6):
                        self.values[idx] = self.read_value(pin)
                        idx += 1
                    elif (idx>6):
                        self.values[idx] = 0.8
                        idx += 1
            except:
                return
            self.msg.header.stamp = rospy.Time.now()
            self.msg.ranges = self.values
            self.pub.publish(self.msg)

            # ir cloud
            pcloud = PointCloud2()
            try:
                for i in range(10):
                    self.ir_cloud[i][0] = (BASE_RADIUS + self.values[i]) \
                                            * cos(self.ir_sensor_angles_rad[i])
                    self.ir_cloud[i][1] = (BASE_RADIUS + self.values[i]) \
                                            * sin(self.ir_sensor_angles_rad[i])
                    self.ir_cloud[i][2] = self.ir_height
            except:
                rospy.logerr("Exeption in ir_cloud calculate!")
                return

            # rospy.loginfo("values: {}\nir_cloud: {}".format(self._list(self.values[5:7]), self._list(self.ir_cloud[5:7],depth=2)))
            # rospy.loginfo("values: {}\nir_cloud: {}".format(round(self.values[6], 2), self._list(self.ir_cloud[6])))
            pcloud.header.frame_id = "base_footprint" ##FIXME: Xem lai frame cho nay
            pcloud = pc2.create_cloud_xyz32(pcloud.header, self.ir_cloud)
            self.ir_pcl_pub.publish(pcloud)


            self.t_next = now + self.t_delta


    def read_value(self, pin):
        value = self.controler.analog_read(pin)

        if value <= 3.0:
            return self.msg.max_range

        try:
            vol = value*(5.0/1023.0)
            distance = 27.728 * pow(vol, -1.2045)
        except:
            return self.msg.max_range

        # convert to metter
        distance /= 100 # distance = distance/100

        if distance > self.msg.max_range: distance = self.msg.max_range
        if distance < self.msg.min_range: distance = self.msg.min_range

        return distance

    def _list(self, list, depth=1):
        if depth == 1:
            return [round(x, 2) for x in list]
        if depth == 2:
            return [[round(x, 2) for x in sublist] for sublist in list]
