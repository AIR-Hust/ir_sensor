#!/usr/bin/env python

# ======================
# Creat by: nv Huy at CIST 
# December 05,2018
# ======================

import rospy
from geometry_msgs.msg import Twist
from ir_sensor.msg import RangeIR_msg

# So luong cam bien su dung
N = 7
vel_linear_x = 0.0
offset = [17, 15, 17, 16, 17, 15, 17]
sonar=[80, 80, 80, 80, 80, 80, 80]

def irEvent(msg):
	global sonar, offset
	sonar=[msg.rangeR0, msg.rangeRF, \
		msg.rangeFR, msg.rangeF0, msg.rangeFL, msg.rangeLF, msg.rangeL0]
	for i in xrange(N):
		sonar[i] = sonar[i] + offset[i]
	# print sonar

def vel_listenner(msg):
    global vel_linear_x
    vel_linear_x = msg.linear.x
    print msg.angular.z 

def check_obstacle(sonar, bubble_boundary):
	global located_obstacle
	obstacle = False
	for i in xrange(N):
		if (sonar[i]<bubble_boundary[i]):
			located_obstacle[i] = 1
			obstacle = True
	if obstacle:
		return True	
	return False

# compute_alpha_R
def compute_alphaR(sonar, located_obstacle, alpha_0):
	sumAngleDis = 0.0
	sumDis = 0.0
	for i in xrange(N):
		sumAngleDis = sumAngleDis + located_obstacle[i]*sonar[i]*alpha_0*(i-N/2)
		sumDis = sumDis + located_obstacle[i]*sonar[i]
	try:
		return sumAngleDis/sumDis
	except ZeroDivisionError:
		pass

if __name__=="__main__":
	rospy.init_node('ir_safety_controller')
	ir_sub_ = rospy.Subscriber('rangeIR_data', RangeIR_msg, irEvent)
	vel_sub_ = rospy.Subscriber('mobile_base/commands/velocity', Twist, vel_listenner)
	pub = rospy.Publisher('ir_cmd_vel', Twist, queue_size=5)

	x = 0
	th = 0 

	located_obstacle = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	bubble_boundary = [0,0,0,0,0,0,0]
	# He so 
	K = [0.1, 0.5, 2.1, 3, 2.1, 0.5, 0.1]; 
	delta_t = 0.30
	alpha_0 = 3.14/6.0
	alpha_R = ''
	obstacle_detected_ = False
	is_dead = 0

	twist = Twist()
	# try:
	while not rospy.is_shutdown():
		for i in xrange(N):
			bubble_boundary[i] = (int(K[i]*100.0*vel_linear_x*delta_t)) + offset[i]
		# print vel_linear_x
		obstacle_detected_ = check_obstacle(sonar,bubble_boundary)

		if obstacle_detected_:
			alpha_R = compute_alphaR(sonar, located_obstacle, alpha_0)
			# print 'sonar' + str(sonar)
			# p1rint 'buble' + str(bubble_boundary)
			# print 'locat' + str(located_obstacle)
			# print 'alpha_R:' + str(alpha_R)
			for i in xrange(N/2-1):
				if located_obstacle[i]==located_obstacle[N-i]:
					is_dead+=1
			if is_dead==0:	
				if alpha_R>0:
					th = -1.0
				else:
					th = 1.0
				control_speed = vel_linear_x*0.9
			else:
				control_speed = vel_linear_x - 0.02
				th = 0.0 
			twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
			pub.publish(twist)
			located_obstacle = [0]*N
			is_dead = 0

	rospy.spin()

	# except Exception as e:
	# 	raise e
	# finally:
	# 	pass





