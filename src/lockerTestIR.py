#!/usr/bin/env python
import rospy
import sys, select, termios, tty
from std_msgs.msg import String

msg = """
Reading from the keyboard and publishing to String
-------------------------------------------------
w - to write
s - to stop write
q - quite
-------------------------------------------------
"""


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('lockerEval', String, queue_size = 1)
    rospy.init_node("locker_ever_ir")
    try:
        print msg
        while(1):
            key = getKey()
            print key
            pub.publish(key)
            if(key=="q" or key == '\x03'):
                break

    except:
        print e