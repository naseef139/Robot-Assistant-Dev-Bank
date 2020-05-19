#!/usr/bin/env python



import rospy
import cv2
from sensor_msgs.msg import Image  ############
from cv_bridge import CvBridge, CvBridgeError  ##############
from std_msgs.msg import String ###############  


def camera_NSF():
 while True:
    pub = rospy.Publisher("camera_topic", Image, queue_size=10) ######'camera_topic' topic,message type is IMAGE
    rospy.init_node('camera', anonymous=True) ######## name of the node is "camera"
    rate=rospy.Rate(10) ###########
    
    ret, frame = cap.read()
    cv2.imshow('video', frame)
    cv2.waitKey(3)
    print('sending frames cont.......')
    

    
    bridge= CvBridge()    ###############
    ros_image = bridge.cv2_to_imgmsg(frame, "bgr8") ##############
        
        
  
    pub.publish(ros_image) ######
    rate.sleep()#########
        

if __name__ == '__main__':


      cap = cv2.VideoCapture(0)
      
      try:
          camera_NSF()
          
          
      except rospy.ROSInterruptException:
              pass
