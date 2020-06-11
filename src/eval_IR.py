#!/usr/bin/env python

import rospy
from ir_sensor.msg import IR_Array_msg
from std_msgs.msg import String

# open file to write
f = open("testir3.txt", "w+")
write = False
cnt = 0


def eval_ir(data):
    # if ()
    global write, cnt
    if(write):
        cnt += 1
        if(cnt <= 100):
            rospy.loginfo("IR 3: {}".format(data.ranges[3]))
            f.write("{}\n".format(data.ranges[3]))
        else:
            cnt = 0
            write = False
            f.write("\n")

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