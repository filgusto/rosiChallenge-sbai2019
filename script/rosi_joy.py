#!/usr/bin/env python
import rospy
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorCommand
from sensor_msgs.msg import Joy

class RosiNodeClass():

	# class constructor
	def __init__(self):

		# sends a message to the user
		rospy.loginfo('Rosi_joy node started')

		# registering to publishers
		self.pub_arm = rospy.Publisher('/rosi/traction_speed', RosiMovementArray, queue_size=1)
		self.pub_traction = rospy.Publisher('/rosi/arms_speed', RosiMovementArray, queue_size=1)

		# registering to subscribers
		self.sub_joy = rospy.Subscriber('/joy', Joy, self.callback_Joy)

		# eternal loop (until second order)
		while not rospy.is_shutdown():

			arm_command_list = RosiMovementArray()
			traction_command_list = RosiMovementArray()

			# mounting the lists
			for i in range(4):

				arm_command = RosiMovement()
				traction_command = RosiMovement()
				
				# mounting arm command list
				arm_command.nodeID = i+1

				arm_command.velocity = 1

				# appending the command to the list
				arm_command_list.movement_array.append(arm_command)

				# mount traction command list
				traction_command.nodeID = i+1
				traction_command.velocity = 0.3

				# appending the command to the list
				traction_command_list.movement_array.append(traction_command)

			# publishing
			self.pub_arm.publish(arm_command_list)		
			self.pub_traction.publish(traction_command_list)

			# sleeps for a while
			rospy.loginfo('sleeping...')
			rospy.sleep(2.)

		# infinite loop
		#while not rospy.is_shutdown():
			# pass

		# enter in rospy spin
		#rospy.spin()

	# joystick callback function
	def callback_Joy(msg):

		# saving joy commands
		self.axes_lin = msg.axes[1]
		self.axes_ang = msg.axes[0]
		self.trigger_left = msg.axes[2]
		self.trigger_right = msg.axes[5]
		self.button_L = msg.axes[4]
		self.button_R = msg.axes[5]

		

# instaciate the node
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('rosi_example_node', anonymous=True)

	# instantiate the class
	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException: pass

