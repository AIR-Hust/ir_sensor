#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from ros_arduino_msgs.msg import *
from std_msgs.msg import Float32

class IRfilter():
    def __init__(self):
        rospy.init_node('IRfilter', log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)

        self.rate = 10
        self.total = 0
        self.avg = 0

        self.IR_filtered = []
        self.num_sample = None
        self.sensor_rate = []
        # self.ir_filtered_pub = rospy.Publisher("~IR_filtered", )
        self.sensor_params = rospy.get_param("/arduino/sensors")
        for name, params in self.sensor_params.iteritems():
            rospy.loginfo("name: {} - rate: {}".format(name, params['rate']))
            
        topic_list = rospy.get_published_topics("/arduino/sensor/")
        rospy.loginfo("rostopic List: {}".format(topic_list))
    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.loginfo("Shutting down Arduino Node...")

if __name__ == "__main__":
    irfilter = IRfilter()
# rate = 10
# count = 0
# total = 0
# avg = 0
# sensor_BR = rospy.Publisher("sensor/ir_BR_filtered", Float32, queue_size=10)

# # sensor_params = rospy.get_param("/arduino/sensors/ir_front_center/rate")
# # num_sample = sensor_params/rate
# num_sample = 10
# sensor_params = rospy.get_param("~sensors") #, dict({})
# exit()

# def read_distance(data):
#     global total, count, num_sample, avg
#     if count >= num_sample - 1:
#         total += data.range
#         avg = total/num_sample
#         rospy.loginfo("sensor_BR: {}".format(avg))
#         sensor_BR.publish(avg)

#         count = 0
#         total = 0

#     else:
#         total += data.range
#         count += 1
           

# if __name__ == "__main__":
#     rospy.init_node('read_ir', anonymous=True)
#     rospy.loginfo("Num sample: {} - Sensor filtered rate: {}".format(num_sample, rate))
#     rospy.loginfo(sensor_params)
#     for name, params in sensor_params.iteritems():
#         rospy.loginfo("name {}".format(name))
#     ir_br_sub = rospy.Subscriber("/arduino/sensor/ir_front_center", Range, read_distance)
#     # while not rospy.is_shutdown():
#     #     if count >= 10:
#     #         rospy.loginfo("sensor_BR: {}".format(avg))
#     #         sensor_BR.publish(avg)

#     rospy.spin()