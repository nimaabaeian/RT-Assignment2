#! /usr/bin/env python3

# Import necessary libraries
import rospy

from assignment_2_2023.srv import LastTarget, LastTargetResponse


def put_target(req):

	

	res = LastTargetResponse()
	
	
	
	res.target_x = rospy.get_param('/des_pos_x',default=0.0)
	res.target_y = rospy.get_param('/des_pos_y',default=0.0)
	
	
	
	return res

def main():


	# Initialize the node 
	rospy.init_node("node_b_service")
	rospy.loginfo("target node started and ready to give the last target the user entered")
	
	rospy.Service('last_target', LastTarget, put_target) 
	
	
		
	rospy.spin()
	
	
if __name__ == "__main__":

	a = main()
