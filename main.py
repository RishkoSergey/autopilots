#! /usr/bin/python

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from numpy import pi 
from math import atan2, hypot


class Main():
	def __init__(self):
		self.first_pose = Pose()
		self.second_pose = Pose()
		self.rubb_sub = rospy.Subscriber('/michelangelo/pose', Pose, self.set_pose_first)
		self.runner_sub = rospy.Subscriber('/raphael/pose', Pose, self.set_pose_second)
		self.runner_pub = rospy.Publisher('/raphael/cmd_vel', Twist, queue_size = 1)

	def rastojaniye(self):
		return hypot(self.first_pose.x - self.second_pose.x, self.first_pose.y - self.second_pose.y)
					
	def set_pose_first(self, pose):
		self.first_pose = pose


	def set_pose_second(self, pose):
		self.second_pose = pose

      	def dvigeniye(self):
		ang = atan2(self.first_pose.y - self.second_pose.y, self.first_pose.x - self.second_pose.x) - self.second_pose.theta
		msg = Twist()
		msg.angular.z = ang
		msg.linear.x = (pi - abs(ang)) / pi
		self.runner_pub.publish(msg)

	def run(self):
		while not rospy.is_shutdown():
      			if self.rastojaniye() < 0.5 :
				rospy.logerr('DOGNALL')
				self.dvigeniye()
			else:
				self.dvigeniye()
			
		
rospy.init_node('cherepahy')
rospy.wait_for_service('/spawn')
spawn_func = rospy.ServiceProxy('/spawn', Spawn)
res = spawn_func(0.0, 6.0, 3.0, 'raphael')
m = Main()
m.run()