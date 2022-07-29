#!/usr/bin/env python
import roslib

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class Joint:
    def __init__(self, motor_name):
        self.name = motor_name
        self.jta = actionlib.SimpleActionClient(
            '/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()
        char = self.name[0]
        goal.trajectory.joint_names = [
            'soft_robotics_gripper_left_finger_joint1', 'soft_robotics_gripper_right_finger_joint1']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)


def main():
    arm = Joint('gripper')
    arm.move_joint([0.26, 0.26])
    arm.move_joint([0, 0])


if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()
