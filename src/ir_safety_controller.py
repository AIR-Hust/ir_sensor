#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from ir_sensor.msg import IR_Array_msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import thread
import math

# constant:
NUM_SENSOR   = 10
NUM_SS_FRONT = 7
NUM_SS_BACK  = 3
PI           = 3.14
NUM_SAMPLE   = rospy.get_param("/ir_sensor/num_sample")
RATE         = NUM_SAMPLE/rospy.get_param("/ir_sensor/sensor_rate")

# Trong so bubble_boundary
# K = 8.0*[0.5, 1.5, 2.1, 3.2, 2.1, 1.5, 0.5, -1.0, -1.0, -1.0]
K = 15.0
OFFSET = NUM_SENSOR*[0.0]
SENSOR_ANGLES = rospy.get_param("/ir_sensor/angles")
SENSOR_ANGLES_RAD = [x*math.pi/180.0 for x in SENSOR_ANGLES]
URGENT_DISTANCE = 0.30
URGENT_BD = NUM_SENSOR*[URGENT_DISTANCE]

ARG = 1 # Lua chon thuat toan su dung
        # ARG=1: test voi bb co dinh = 30cm

class IR_safety_Controller():
    def __init__(self):
        rospy.init_node('ir_safety_controller', log_level=rospy.DEBUG)
        self.ir_array_sub = rospy.Subscriber("IR_sensor_arr", IR_Array_msg, self.ir_array_filter)
        self.odom_vel_sub = rospy.Subscriber("odom", Odometry, self.read_odom_vel)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.read_cmd_vel)
        self.ir_cmd_vel_pub = rospy.Publisher("/ir_cmd_vel", Twist, queue_size=10)

        self.count = 0
        self.ir_array_sum = NUM_SENSOR*[0]
        self.ir_array_filtered = NUM_SENSOR*[0.80]
        self.ir_front = self.ir_array_filtered[0:NUM_SS_FRONT]
        self.ir_back = self.ir_array_filtered[NUM_SS_FRONT:NUM_SENSOR]

        self.odom_vel_x = 0.0
        self.cmd_vel_x = 0.0
        self.cmd_vel_th = 0.0

        self.bubble_boundary = NUM_SS_FRONT*[0.80]
        self.faced_obstacle = False

        self.obstacle_located = NUM_SS_FRONT*[0]

        self.mutex = thread.allocate_lock()

        self.t_delta = rospy.Duration(1.0/RATE)
        self.t_next = rospy.Time.now() + self.t_delta
        rospy.loginfo("NUM_SAMPLE: {}".format(NUM_SAMPLE))
        rospy.loginfo("SENSOR_ANGLES_RAD: {}".format(self._list(SENSOR_ANGLES_RAD)))

        self.twist = Twist()
        self.ctr_vel_x = 0
        self.turn_z = 0

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if now > self.t_next:
                try:
                    # if(ARG==1):
                    #     self.bubble_boundary = NUM_SENSOR*[URGENT_DISTANCE]
                    # elif(ARG==2):
                    self.bubble_boundary = [K*self.odom_vel_x*(1.0/RATE)*math.cos(r)\
                                            for r in SENSOR_ANGLES_RAD]
                    # print(self.bubble_boundary)
                    self.bubble_boundary = [(bb if bb < 0.70 else 0.70) \
                                            for bb in self.bubble_boundary]
                    # print log every 2s
                    rospy.logdebug_throttle(2, "cmd_vel_x: {} | bb: {}"\
                                .format(self.odom_vel_x ,self._list(self.bubble_boundary)))
                    #FIXME: xem lại hệ số k, vận tốc
                    obstacle = self.check_obstacle()
                    # rospy.logdebug(obstacle)

#                    print(obstacle[0])
                    if (obstacle[0] == 'no_obstacle'):
                        continue
                    elif (obstacle[0] == 'urgent'):
                        if (obstacle[1]):
                            self.stop_goForward()
                        if (obstacle[2]):
                            self.stop_goBackward()
                        continue
                    else: #(obstacle[0] == 'bubble_obstacle'):
                        obstacle_dir = self.obstacle_directions()
                        rospy.loginfo("Faced obstacle at {}".format(obstacle_dir))
                        rospy.logdebug("bubble_boundary {}".format(self._list(self.bubble_boundary)))
                        rospy.logdebug("ir_array_filtered {}".format(self._list(self.ir_array_filtered)))
                        if (obstacle_dir == "Left"):
                            self.turn_right()
                       #      self.force_stop()
                        else:
                            self.turn_left()
                       #      self.force_stop()
                except Exception as e:
                    rospy.logerr(e)
                    return
                self.t_next = now + self.t_delta

    def check_obstacle(self):
        obstacle = ['no_obstacle', 0, 0]
        try:
            urgent = [(1 if x < u else 0) for x,u in zip(self.ir_array_filtered, URGENT_BD)]
            if (1 in urgent):
                obstacle[0] = 'urgent'
                if (1 in urgent[0:NUM_SS_FRONT]):
                    rospy.loginfo("Urgent front! Front: {}".format(self._list(self.ir_front)))
                    obstacle[1] = 1
                if (1 in urgent[NUM_SS_FRONT:NUM_SENSOR]):
                    rospy.loginfo("Urgent back! Back: {}".format(self._list(self.ir_back)))
                    obstacle[2] = 1
                return obstacle
            else:
                rospy.logdebug_throttle(1,self.bubble_boundary[0:NUM_SS_FRONT])
                self.obstacle_located = [(1 if x < bb else 0) for x,bb in \
                                         zip(self.ir_front, \
                                             self.bubble_boundary[0:NUM_SS_FRONT])]
                if (1 in self.obstacle_located):
                    rospy.logdebug("obstacle_locate: {}".format(self.obstacle_located))
                    obstacle[0] = 'bubble_obstacle'
                    obstacle[1] = 1
                    # return obstacle
        except Exception as e:
            rospy.logerr(e)
        finally:
            return obstacle

    def turn_left(self):
        rospy.loginfo("Turn left")
        self.ctr_vel_x = (self.cmd_vel_x - 0.2 if self.cmd_vel_x > 0.2 else 0)
        self.turn_z = 2.0
        self.twist = self.set_twist_cmd(self.ctr_vel_x, self.turn_z)
        self.ir_cmd_vel_pub.publish(self.twist)

    def turn_right(self):
        rospy.loginfo("Turn right")
        self.ctr_vel_x = (self.cmd_vel_x - 0.2 if self.cmd_vel_x > 0.2 else 0)
        self.turn_z = -2.0
        self.twist = self.set_twist_cmd(self.ctr_vel_x, self.turn_z)
        self.ir_cmd_vel_pub.publish(self.twist)

    def force_stop(self):
        rospy.loginfo("Force stop!")
        self.twist = self.set_twist_cmd(0.0, 0.0)
        self.ir_cmd_vel_pub.publish(self.twist)
        ### Add vao de ko bi dung mai
        #r = rospy.Rate(10)
        #r.sleep
        #check_obstacle()
        ### het sua (Tung)

    def stop_goForward(self):
        rospy.loginfo("Cannot go forward!")
        if(self.odom_vel_x > 0):
            self.force_stop()

    def stop_goBackward(self):
        rospy.loginfo("Cannot go backward!")
        if(self.odom_vel_x < 0):
            self.force_stop()

    def set_twist_cmd(self, ctr_vel_x, turn_z):
        twist = Twist()
        twist.linear.x = ctr_vel_x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = turn_z
        return twist

    def obstacle_directions(self):
        try:
            obstacles_right = sum(self.obstacle_located[0:4])
            obstacles_left = sum(self.obstacle_located[4:7])

            return "Left" if obstacles_left > obstacles_right else "Right"

        except Exception as e:
            rospy.logerr(e)
            return

    def ir_array_filter(self, data):
        if self.count < NUM_SAMPLE:
            self.ir_array_sum = [x+y for x,y in zip(self.ir_array_sum, data.ranges)]
            # print("count: " + str(self.count) + " ir_arr_sum: " + str(self.ir_array_sum))

            self.count += 1
        else:
            self.ir_array_filtered[:] = [x/NUM_SAMPLE for x in self.ir_array_sum]
            self.ir_front = self.ir_array_filtered[0:NUM_SS_FRONT]
            self.ir_back = self.ir_array_filtered[NUM_SS_FRONT:NUM_SENSOR]
            rospy.logdebug_throttle(1,"ir_array_filtered: {}".format(self._list(self.ir_array_filtered)))
            self.count = 0
            self.ir_array_sum = NUM_SENSOR*[0]

    def  read_odom_vel(self, msg):
        self.odom_vel_x = msg.twist.twist.linear.x

    def read_cmd_vel(self, msg):
        self.cmd_vel_x = msg.linear.x
        self.cmd_vel_th = msg.angular.z

    def _list(self, list):
        return [round(x, 2) for x in list]
if __name__ == "__main__":
    ir_safety_controller = IR_safety_Controller()
