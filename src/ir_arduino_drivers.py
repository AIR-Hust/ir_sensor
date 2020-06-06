#!/usr/bin/env python

"""
IR sensor for arduino IR sensor reader
Created for EAI robot expand by nvHuy
Feb 23, 2020
"""

import roslib; roslib.load_manifest("ros_arduino_python")
import rospy
from ir_sensor.msg import IR_Array_msg

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

        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            try:
                idx = 0
                for pin in self.pins:
                    print(self.values)
                    if (idx<=6):
                        self.values[idx] = self.read_value(pin)
                        idx += 1
                    elif (idx>6) and (idx<10):
                        self.values[idx] = 0.8
                        idx += 1
            except:
                return
            self.msg.header.stamp = rospy.Time.now()
            self.msg.ranges = self.values
            self.pub.publish(self.msg)

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
