#!/usr/bin/python
import sys
import rospy
import math
import time
from pan_and_tilt_control.msg import PanAndTilt



class PanAndTiltClient(object):

    def __init__(self):
        rospy.loginfo("Starting PanAndTiltClient...")

        self._pan_and_tilt_pub = rospy.Publisher('/pan_and_tilt', PanAndTilt, queue_size=1)

        rospy.loginfo("PanAndTiltServer...READY")

    def pan_and_tilt_move(self, pan, tilt):
        """
        Topic Publisher
        """
        pan_and_tilt_msg = PanAndTilt()
        pan_and_tilt_msg.pan = pan
        pan_and_tilt_msg.tilt = tilt
        rospy.loginfo("Sending PandAndTilt New msg: " + str(pan_and_tilt_msg))
        self._pan_and_tilt_pub.publish(pan_and_tilt_msg)


def yaw_pitch_circle_test(repetitions=1):

    pan_and_tilt_client_obj = PanAndTiltClient()

    # These values are base on Hardware observations where they are stable.
    max_range = 100
    min_range = 30
    period = 0.1
    increments = 10

    for num in range(repetitions):
        for angle in range(0,359,increments):
            pitch_angle = int((((math.sin(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)
            yaw_angle = int((((math.cos(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)

            rospy.loginfo("Moving Yaw="+str(yaw_angle))
            rospy.loginfo("Moving Pitch=" + str(pitch_angle))

            pan_and_tilt_client_obj.pan_and_tilt_move(pan=yaw_angle, tilt=pitch_angle)
            time.sleep(period)

        for angle in range(0,359,-increments):
            pitch_angle = int((((math.sin(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)
            yaw_angle = int((((math.cos(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)

            print("Moving Yaw="+str(yaw_angle))
            print("Moving Pitch=" + str(pitch_angle))

            pan_and_tilt_client_obj.pan_and_tilt_move(pan=yaw_angle, tilt=pitch_angle)
            time.sleep(period)

def raw_input_pan_and_tilt_test():
    pan_and_tilt_client_obj = PanAndTiltClient()

    while not rospy.is_shutdown():
        print(">>>>>>New Input>>>>>>>>>>\n")
        pan = float(raw_input("Input Pan=>"))
        tilt = float(raw_input("Input Tilt=>"))
        pan_and_tilt_client_obj.pan_and_tilt_move(pan=pan, tilt=tilt)
        print(">>>>>>@@@@@@@@@>>>>>>>>>>\n")


if __name__ == "__main__":
    rospy.init_node('pan_and_tilt_client')
    #yaw_pitch_circle_test(repetitions=2)
    raw_input_pan_and_tilt_test()

