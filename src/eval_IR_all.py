#!/usr/bin/env python

import rospy
from ir_sensor.msg import IR_Array_msg
from std_msgs.msg import String
from datetime import datetime

numIR = 7
# open file to write
now = datetime.now()
filename_time = now.strftime("%Y%m%d_%H%M%S")
LOG_FILE = "/home/ubuntu/catkin_ws/src/ir_sensor/src/log/eval_IR_all_{}.txt".format(filename_time)
f = open(LOG_FILE, "w+")
write = False
view = False
cnt = 0


def eval_ir(data):
    # if ()
    global write, cnt, view
    if(write):
        rospy.loginfo("IR : {}".format(data.ranges))
        f.write("{}\n".format(data.ranges))
    if(view):
        rospy.loginfo("IR : {}".format(data.ranges))

def setkey(key):
    global write
    if(key.data == "w"):
        write = True
        rospy.loginfo("key: {}".format(key))
    if(key.data == "s"):
        f.write("\n")
        write = False
    if(key.data == "q"):
        exit()
    if (key.data == "v"):
        view = True
    if(key.data == "c"):
        rospy.loginfo("Closing file")
        f.close()

def shutdown():
    f.close()

rospy.init_node("Eval_IR", log_level = rospy.INFO)
rospy.on_shutdown(shutdown)

sub = rospy.Subscriber("IR_sensor_arr", IR_Array_msg, eval_ir)
subkey = rospy.Subscriber("lockerEval", String, setkey)

rospy.spin()
