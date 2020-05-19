#!/usr/bin/env python

import pyttsx3

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
import re

def callback(data):
    no= data.data

    print("######## recieved ############")
    
    if no:
        print(no)

        engine = pyttsx3.init()
        engine.say("its  ok  thank  you very much and have a nice day")
        engine.runAndWait()

             
def listener2():
    rospy.init_node('id_no_recieve')
    rospy.Subscriber("id_number", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener2()

