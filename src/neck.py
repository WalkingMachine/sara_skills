import rospy

import math

from geometry_msgs.msg import PointStamped


class Neck():
    def __init__(self):
        self._goal = None
        self._at_setpoint = False
        self.sara_height = 1.69

    # Move neck downward to help with navigation
    def move_neck_navigation(self):
        # Without ros_control the correct value is float64 with -1
        # With ros_control the angle to set is to be determined
        set_neck_position(0, -30)

    # Move neck to intial position
    def reset_neck_head(self):
        set_neck_position(0,0)

    # Move neck to look at the arm
    def move_neck_to_arm(self):
        set_neck_position(30, -30)

    def set_neck_position(self, position_x, position_y):
        pass

    # Set sara robot neck so that the robot make eye contact with a person at 1
    # meters from her. The degres need is 0.12 degres
    def look_at_someone(self, tf):
        set_neck_position(0, math.degrees(math.atan(1.79 - self.sara_height))


    if __name__ == "__main__":
        rospy.init_node('sara_neck_executionner', anonymous=True)
        neck = Neck()
