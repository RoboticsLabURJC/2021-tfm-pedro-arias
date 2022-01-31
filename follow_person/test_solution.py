#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

import math

def distance(a, b):
    '''
    2D euclidean distance
    a: Pose
    b: Pose
    '''
    return math.sqrt(math.pow(abs(b.position.x) - abs(a.position.x), 2) 
    	+ math.pow(abs(b.position.y) - abs(a.position.y), 2))

def callback(msg):
	idx_iris = msg.name.index("iris")
	idx_person = msg.name.index("static_person")
	iris_pub.publish(msg.pose[idx_iris])
	person_pub.publish(msg.pose[idx_person])

	d = distance(msg.pose[idx_iris], msg.pose[idx_person])
	dist_pub.publish(d)
	error_pub.publish(abs(4.5-d))

rospy.init_node('solution_tester')
rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
iris_pub = rospy.Publisher('/tester/iris', Pose, queue_size=10)
person_pub = rospy.Publisher('/tester/person', Pose, queue_size=10)
dist_pub = rospy.Publisher('/tester/dist', Float64, queue_size=10)
error_pub = rospy.Publisher('/tester/error', Float64, queue_size=10)

rospy.spin()