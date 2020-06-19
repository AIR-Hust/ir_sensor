#!/usr/bin/env python

import rospy
from ir_sensor.msg import IR_Array_msg
from std_msgs.msg import String
from datetime import datetime

numIR = 3
NumSample = 200

# open file to write
now = datetime.now()
filename_time = now.strftime("%Y%m%d_%H%M%S")
LOG_FILE = "log/eval_IR{}_{}.txt".format(numIR, filename_time)
f = open(LOG_FILE, "w+")
write = False
view = False
cnt = 0

f.write("Test IR number: {}\n".format(numIR))

def eval_ir(data):
    # if ()
    global write, cnt, view
    if(write):
        view = False
        cnt += 1
        if(cnt <= NumSample):
            rospy.loginfo("IR numIR: {}".format(data.ranges[numIR]))
            f.write("{}\t".format(data.ranges[numIR]))
        else:
            cnt = 0
            write = False
            f.write("\n")
            rospy.loginfo("\nWrite done!")
    if (view):
        rospy.loginfo("IR numIR: {}".format(data.ranges[numIR]))

def setkey(key):
    global write, view
    if(key.data == "w"):
        write = True
        rospy.loginfo("key: {}".format(key))
    if(key.data == "s"):
        f.write("\n")
        write = False
    if(key.data == "q"):
        exit()
    if(key.data == "c"):
        rospy.loginfo("Closing file")
        f.close()
    if(key.data == "v"):
        view = True
    if(key.data == "h"):
        view = False

def shutdown():
    f.close()

rospy.init_node("Eval_IR", log_level = rospy.INFO)
rospy.on_shutdown(shutdown)

sub = rospy.Subscriber("IR_sensor_arr", IR_Array_msg, eval_ir)
subkey = rospy.Subscriber("lockerEval", String, setkey)
rospy.loginfo("Test IR number: {}\n".format(numIR))

rospy.spin()