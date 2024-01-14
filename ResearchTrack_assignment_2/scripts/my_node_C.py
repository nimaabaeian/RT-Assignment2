#! /usr/bin/env python3

# Import necessary libraries
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import LastTarget, LastTargetRequest
from assignment_2_2023.srv import Average, AverageResponse
import math

rospy.wait_for_service("last_target")
client = rospy.ServiceProxy('last_target', LastTarget)
# Create a LastTargetRequest (empty request in this case)
request = LastTargetRequest()



velocity_list = list()
distance = 0
average_vel_x = 0

def give_avg(req):
	global  distance, average_vel_x

	res = AverageResponse()
	
	res.dist = distance
	res.velocity_mean = average_vel_x
	
	return res
	

def get_average(msg):

	global response, velocity_list, distance, average_vel_x
	
	# Call the LastTarget service and get the response
	response = client(request)

	target_x = response.target_x
	target_y = response.target_y
	
	Window = rospy.get_param('average_window')
	
	x_now = msg.x
	y_now = msg.y
	
	vel_x_now = msg.vel_x
	
	
	distance = math.sqrt((target_x - x_now)**2 + (target_y - y_now)**2)
	
	velocity_list.append(msg.vel_x)
	
	if len(velocity_list) < Window:
	
		average_vel_x = sum(velocity_list) / len(velocity_list)
	
	else:
	
		average_vel_x = sum(velocity_list[-Window:])/Window
		
	#rospy.loginfo("distance %f and velocity %f" , distance, average_vel_x)
	
	


def main():


	# Initialize the node 
	rospy.init_node("node_c_service")
	rospy.loginfo("Node started and ready to calculate the average")
	rospy.Service('average', Average, give_avg)
	
	rospy.Subscriber("/kinematics", Vel, get_average)
	
	
	
	rospy.spin()
	
	
if __name__ == "__main__":

	main()
