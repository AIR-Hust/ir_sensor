#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from datetime import datetime


# open file to write
now = datetime.now()
filename_time = now.strftime("%Y%m%d_%H%M%S")
LOG_FILE = "/home/ubuntu/catkin_ws/src/ir_sensor/src/log/eval_odom_{}.txt".format(filename_time)
f = open(LOG_FILE, "w+")
write = False
view = False
cnt = 0


def eval_odom(data):
    # if ()
    global write, cnt, view
    if(write):
        rospy.loginfo("Pose and Velocity: {} {} {} {}".format(data.pose.pose.position.x,data.pose.pose.position.y, data.twist.twist.linear.x, data.twist.twist.linear.y))
        f.write("\n{} {}".format(data.pose.pose.position.x,data.pose.pose.position.y, data.twist.twist.linear.x, data.twist.twist.linear.y))
    if(view):
        rospy.loginfo("Pose and Velocity: {}\n{}".format(data.pose.pose.position,data.twist.twist.linear))
def setkey(key):
    global write
    if(key.data == "w"):
        write = True
        rospy.loginfo("key: {}".format(key))
    if (key.data == "v"):
        view = True
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

rospy.init_node("Eval_Odom", log_level = rospy.INFO)
rospy.on_shutdown(shutdown)

sub = rospy.Subscriber("odom", Odometry, eval_odom)
subkey = rospy.Subscriber("lockerEval", String, setkey)

rospy.spin()