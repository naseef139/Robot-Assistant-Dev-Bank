#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from pan_and_tilt_client import PanAndTiltClient
from icog_face_tracker.msg import faces

class FaceTracker(object):
    def __init__(self):
        rospy.loginfo("Start Face Tracker Init process...")

        self.pan_and_tilt_client_obj = PanAndTiltClient()

        rospy.loginfo("Start camera suscriber...")
        self.faces_msg = None
        self._faces_detection_topic = "/faces"
        self._check_facedetection_ready()
        self.image_sub = rospy.Subscriber(self._faces_detection_topic, faces, self.faces_detection_callback)

        rospy.loginfo("Finished Face Tracker Init process...Ready")

    def _check_facedetection_ready(self):

        while self.faces_msg is None and not rospy.is_shutdown():
            try:
                self.faces_msg = rospy.wait_for_message(self._faces_detection_topic, faces, timeout=1.0)
                rospy.logdebug("Current "+str(self._faces_detection_topic+" READY=>"))
            except:
                rospy.logerr("Current "+str(self._faces_detection_topic+" not ready yet, retrying..."))
        rospy.logdebug("facedetection_ready READY")

    def face_follow(self):
        """
        It first sets the PanAndTilt to degrees P&T= [0,0]
        :return:
        """
        pan_tilt_pose = [0.0, 0.0]
        self.pan_and_tilt_client_obj.pan_and_tilt_move(pan=pan_tilt_pose[0],
                                                       tilt=pan_tilt_pose[1])

        # We now read the Face that should be infront
        # We get the position of the person face ( that shouldnt move )

        # Start moving incrementaly pan and tilt until the deltas are practicaly zero
        pan_and_tilt_increment = 1.0
        # This is due to the fact that the image coordinates might not be the same as rotations.
        pan_rotation_sign = -1.0
        tilt_rotation_sign = 1.0
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            delta_width, delta_height = self.get_delta_face()
            sign_delta_width = np.sign(delta_width)
            sign_delta_height = np.sign(delta_height)
            pan_tilt_pose[0] += sign_delta_width * pan_rotation_sign * pan_and_tilt_increment
            pan_tilt_pose[1] += sign_delta_height * tilt_rotation_sign * pan_and_tilt_increment

            rospy.loginfo("######################")
            rospy.loginfo("delta_width"+str(delta_width))
            rospy.loginfo("delta_height"+str(delta_height))
            rospy.loginfo("pan_tilt_pose" + str(pan_tilt_pose))
            rospy.loginfo("######################")
            self.pan_and_tilt_client_obj.pan_and_tilt_move(pan=pan_tilt_pose[0],tilt=pan_tilt_pose[1])
            rate.sleep()

    def get_delta_face(self):
        important_face = None
        image_width = None
        image_height = None
        delta_pan = None
        delta_tilt = None

        #rate = rospy.Rate(10.0)
        while important_face is None and not rospy.is_shutdown():
            important_face, image_width, image_height = self.check_face_detected()
            if important_face is not None:
                break
            #else:
                #rate.sleep()

        delta_width, delta_height = self.calculate_delta_face(important_face, image_width, image_height)

        return delta_width, delta_height

    def check_face_detected(self):
        image_width = self.faces_msg.image_width
        image_height = self.faces_msg.image_height
        important_face = self.get_bigger_face(self.faces_msg.face_boxes)
        return important_face, image_width, image_height

    def calculate_delta_face(self, important_face, image_width, image_height):

        face_width = important_face.width
        face_height = important_face.height
        face_top = important_face.top
        face_left = important_face.left
        face_width_center = face_left + (face_width / 2.0)
        face_height_center = face_top + (face_height / 2.0)

        center_width = image_width / 2.0
        center_height = image_height / 2.0
        delta_width = face_width_center - center_width
        delta_height = face_height_center - center_height

        return delta_width, delta_height

    def get_bigger_face(self, face_boxes):
        """
        The biger the area in theory the closer it is.
        :return:
        """
        index = 0
        bigger_index = 0
        bigger_area = 0.0
        for face in face_boxes:
            area = face.width * face.height
            if area > bigger_area:
                bigger_area = area
                bigger_index = index
            index += 1

        if len(face_boxes) > 0:
            biggest_face = face_boxes[bigger_index]
        else:
            biggest_face = None

        return biggest_face


    def faces_detection_callback(self, data):
        self.faces_msg = data




def main():
    rospy.init_node('face_tracker_node', anonymous=True, log_level=rospy.DEBUG)
    face_tracker_obj = FaceTracker()
    face_tracker_obj.face_follow()
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

