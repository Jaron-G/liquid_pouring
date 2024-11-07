import signal
import sys

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class RobotMover:
    """Small node to move the robot"""

    def __init__(self):
        super(RobotMover, self).__init__()

        rospy.init_node("velocity_node")
        self.pub = rospy.Publisher(
        "/ur10e_robot/joint_group_vel_controller/command", Float64MultiArray, queue_size=10
        )
        self.speed = 0
        self.msg = Float64MultiArray()
        self.msg.data = [0, 0, 0, 0, 0, self.speed]
        self.msg.layout.data_offset = 1

    def set_wrist3_speed(self,speed):
        self.speed = speed
        self.msg.data = [0, 0, 0, 0, 0, self.speed]
        self.pub.publish(self.msg)

    def signal_handler(self, sig, frame):
        rospy.loginfo("You pressed Ctrl+C!")
        self.msg.data = [0, 0, 0, 0, 0, 0.0]
        self.pub.publish(self.msg)
        sys.exit(0)


if __name__ == "__main__":

    robot_mover = RobotMover()
    rate = rospy.Rate(10)

    signal.signal(signal.SIGINT, robot_mover.signal_handler)

    while not rospy.is_shutdown():
        robot_mover.set_wrist3_speed(0.4)
        rospy.sleep(2)
        robot_mover.set_wrist3_speed(-0.4)
        rospy.sleep(2)
