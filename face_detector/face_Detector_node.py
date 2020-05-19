#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('my_package')
import sys
import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import cv2
from std_msgs.msg import String

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy



bridge = CvBridge()

def image_callback(msg):
    
    pub = rospy.Publisher('2D_coordinates', numpy_msg(Floats),queue_size=10)
 
        # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")



    gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(
        gray,
        scaleFactor=2,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.CASCADE_SCALE_IMAGE
        )
        
    for (x, y, w, h) in faces:
           
        rect = cv2.rectangle(cv2_img, (x, y), (x+w, y+h), (0, 0, 255), 2)

        cv2.line(rect,(x+w/2,y),(x+w/2,y+h),(255,255,255),1)

        cv2.line(cv2_img,(320,0),(320,480),(255,255,255),3)

        a = numpy.array([x+w/2, y+h/2], dtype=numpy.float32)
        
        print('inter connecting..........')
        print(len(faces))
        cv2.imshow("pic3", cv2_img)
        cv2.waitKey(3) 
 
        pub.publish(a) 
def main_sub():

    rospy.init_node('face_detection')
    # Define your image topic
    
    # Set up your subscriber and define its callback
    
    rospy.Subscriber("camera_topic",Image,image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':

      face_cascade = cv2.CascadeClassifier("/home/naseef/OpenCV/data/haarcascades/haarcascade_frontalface_default.xml")
   
      try:
          main_sub()
          
          
      except rospy.ROSInterruptException:
              pass
