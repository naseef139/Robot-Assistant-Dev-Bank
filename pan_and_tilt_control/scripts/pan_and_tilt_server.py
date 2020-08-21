#!/usr/bin/python
import sys
import rospy
from pan_and_tilt_control.msg import PanAndTilt



class PanAndTiltServer(object):
    """
    Class for using the Pan and Tilt, real or simulated.
    This allows us to launch always the same node, and just
    initialise the move PanAndTilt object for the Real pan and tilt or
    for the simulated version in Gazebo. This way we reuse code and
    we force both classes to be transparent and have the same interface.
    """

    def __init__(self, is_simulated = False, inside_raspberry = True):
        rospy.loginfo("Starting PanAndTiltServer...")

        self._inside_raspberry = inside_raspberry
        self._is_simulated = is_simulated
        # We Initialise the Real or Simulated Control object
        if self._is_simulated:
            from pan_and_tilt_move_sim import PanAndTiltMoveSim
            self._pan_and_tilt_move_object = PanAndTiltMoveSim()
            rospy.loginfo("PanAndTiltServer SIMULATED___STARTED")
        else:
            if inside_raspberry:
                from pan_and_tilt_move import PanAndTiltMove
                self._pan_and_tilt_move_object = PanAndTiltMove()
                rospy.loginfo("PanAndTiltServer REAL___STARTED")

        pan_and_tilt_sub = rospy.Subscriber('/pan_and_tilt', PanAndTilt, self.pan_and_tilt_callback)

        rospy.loginfo("PanAndTiltServer...READY")

    def pan_and_tilt_callback(self, msg):
        """
        Topic Subscriber callback
        """

        rospy.logdebug("Received PandAndTilt New msg: " + str(msg))
        if self._inside_raspberry or self._is_simulated:
            self._pan_and_tilt_move_object.move_to_pitch_yaw(yaw=msg.pan, pitch=msg.tilt)

if __name__ == "__main__":
    rospy.init_node('pan_and_tilt_server')

    if len(sys.argv) > 2:
        simulated = sys.argv[1]
        is_raspberry = sys.argv[2]

        is_simulation = simulated == "simulated"
        in_raspberry = is_raspberry == "true"

        machine = PanAndTiltServer(is_simulated=is_simulation, inside_raspberry=in_raspberry)
        rospy.spin()
    else:
        rospy.logerr("Use: python pan_and_tilt_control pan_and_tilt_server.py simulated/real true(is raspberry or not)")

