import rospy

from geometry_msgs.msg import PointStamped


class Neck():
    def __init__(self):
        self._goal = None
        self._at_setpoint = False

    # Move neck downward to help with navigation
    def move_neck_navigation(self):
        # Without ros_control the correct value is float64 with -1
        # With ros_control the angle to set is to be determined
        pass

    # Move neck to intial position
    def reset_neck_head(self):
        pass

    # Move neck to look at the arm
    def move_neck_to_arm(self):
        pass

    def set_neck_position(self):
        pass


    if __name__ == "__main__":
        rospy.init_node('sara_neck_executionner', anonymous=True)
        neck = Neck()
