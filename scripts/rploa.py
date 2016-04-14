#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import serial
import struct
import math


withdraw_dis = 0.0  # Distance to withdraw
disregard_dis = 0.0 # Distance to disregard pilot input
min_valid_range = 100.0 # Minimum valid distance

ser = serial.Serial()
def callback(scan):

	global withdraw_dis, disregard_dis, min_valid_range, ser

	min_idx = 0
	min_rng = 10000.0
	
	# Find the closest object
	for i in range(0, 360):
		rng = scan.ranges[i]
		if  min_valid_range < rng and rng < min_rng:
			min_idx = i
			min_rng = rng

	flag = ''
	
	if min_rng < withdraw_dis:
		flag = 'w'
	elif min_rng < disregard_dis:
		flag = 'd'

	else:
		flag = 'c'
		
	#calculate the direction in radian
	direction = min_idx / 360.0 * math.pi

	packet = struct.pack("<cffc", flag, direction, min_rng, '%')
	
	ser.write(packet)
	ser.flush()

	rospy.loginfo("%f %fm", direction, min_rng)

    
def listener():

	rospy.init_node('obstacle_avoidance', anonymous=True)
	
	global withdraw_dis, disregard_dis, min_valid_range, ser
	
	withdraw_dis = rospy.get_param('~withdraw_dis')
	disregard_dis = rospy.get_param('~disregard_dis')
	min_valid_range = rospy.get_param('~min_valid_range')

	ser = serial.Serial(rospy.get_param('~port'), 115200, timeout=1)

	rospy.Subscriber("scan", sensor_msgs.msg.LaserScan, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()