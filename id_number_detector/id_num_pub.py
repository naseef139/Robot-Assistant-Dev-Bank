#!/usr/bin/env python


import cv2
import pytesseract
import re

import rospy
from std_msgs.msg import String
import numpy as np


def gray_scale():
 while(True):
    pub = rospy.Publisher("id_number", String, queue_size=10) 
    rospy.init_node('id_no_send', anonymous=True)
     

    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kernel = np.ones((5,5),np.float32)/25
    gray1 = cv2.filter2D(gray,-1,kernel)
    blurred = cv2.GaussianBlur(gray1, (5, 5), 0)
    edges = cv2.Canny(blurred,400,600,apertureSize = 5)
    

    cv2.imshow('frame', gray1)       
 

    print("sending......")


    text = pytesseract.image_to_string(gray1)

    result = re.match(r'\d\d\d\d\d\d\d\d\d', text)
     
    

    if result:
	pub.publish(text)
        print(text)
        cap.release()
    

    
	
    if cv2.waitKey(1) & 0xFF == ord('q'):
	break



if __name__ == '__main__':

      pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

      cap = cv2.VideoCapture(0)

      try:
          gray_scale()
          
          
      except rospy.ROSInterruptException:
              pass

