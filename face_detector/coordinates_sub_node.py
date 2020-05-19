#!/usr/bin/env python


import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

def callback(data):
    b= data.data
    print "x:",b[0]
    print "y:",b[1]
    print " ----------- "
             
def coordinates():
    rospy.init_node('coordinate_recieve')
    rospy.Subscriber("2D_coordinates", numpy_msg(Floats), callback)
    rospy.spin()

if __name__ == '__main__':

   coordinates()
