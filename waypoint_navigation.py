#!/usr/bin/env python3



from locale import currency
from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():

	def __init__(self):
		
		rospy.init_node('drone_control')	

		self.drone_position = [0.0,0.0,0.0]

		self.points = [[0,0,23], [2,0,23], [2,2,23], [-2,2,23], [-2,-2,23], [2,-2,23], [2,0,23], [0,0,23]]

		self.setpoint = self.points[0]

		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		self.Kp = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
		self.Ki = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
		self.Kd = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

		self.curr_errors , self.prev_errors, self.sum_errors = [0,0,0], [0,0,0], [0,0,0]
		
		self.min_values, self.max_values = [1000,1000,1000], [2000,2000,2000]

		self.proportionals, self.derivatives, self.integrations = 0, 0, 0

		self.sample_time = 0.033 


		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.alt_error_pub = rospy.Publisher('/alt_error',Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error',Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error',Float64, queue_size=1)


		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)


		self.arm() 



	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)



	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)



	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z



	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.06
		self.Ki[1] = pitch.Ki * 0.008
		self.Kd[1] = pitch.Kd * 0.3

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.06
		self.Ki[0] = roll.Ki * 0.008
		self.Kd[0] = roll.Kd * 0.3



	def pid(self):

		for x in range(8):
			self.setpoint = self.points[x]
			for i in range(3):
				self.curr_errors[i] = self.setpoint[i] - self.drone_position[i]
				self.proportionals = self.curr_errors[i] * self.Kp[x][i]
				self.derivatives = ((self.curr_errors[i] - self.prev_errors[i]) / self.sample_time ) * self.Kd[x][i]
				self.integrations = (self.sum_errors[i] * self.sample_time) * self.Ki[x][i]


				if i == 0:
					self.cmd.rcRoll = int(1500 - (self.proportionals + self.derivatives - self.integrations) )
					
				elif i == 1:
					self.cmd.rcPitch = int(1500 + (self.proportionals + self.derivatives - self.integrations) )
					
				elif i == 2:
					self.cmd.rcThrottle = int(1500 - (self.proportionals + self.derivatives - self.integrations) )

				
				if self.cmd.rcThrottle > self.max_values[2]:
					self.cmd.rcThrottle = self.max_values[2]

				if self.cmd.rcThrottle < self.min_values[2]:
					self.cmd.rcThrottle = self.min_values[2]

				if self.cmd.rcPitch >self.max_values[1]:
					self.cmd.rcPitch = self.max_values[1]

				if self.cmd.rcPitch < self.min_values[1]:
					self.cmd.rcPitch = self.min_values[1]

				if self.cmd.rcRoll >self.max_values[0]:
					self.cmd.rcRoll = self.max_values[0]

				if self.cmd.rcRoll < self.min_values[0]:
					self.cmd.rcRoll = self.min_values[0]



				self.prev_errors[i] = self.curr_errors[i]
				self.sum_errors[i] += self.curr_errors[i]


				if self.curr_errors[0] <= 0.1 and self.curr_errors[1] <= 0.1 and self.curr_errors[2] <= 0.1 and self.curr_errors[0] >= -0.1 and self.curr_errors[1] >= -0.1 and self.curr_errors[2] >= -0.1 :
					print(self.setpoint )
					print("done... ")
					break
			


		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.curr_errors[2])
		self.pitch_error_pub.publish(self.curr_errors[1])
		self.roll_error_pub.publish(self.curr_errors[0])
		self.proportionals_pub.publish(self.proportionals)
		self.integrations_pub.publish(self.integrations)
		self.derivatives_pub.publish(self.derivatives)



if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(30)

	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()