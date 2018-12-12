#!/usr/bin/env python2

import rospy
import actionlib
import std_msgs.msg
import sensor_msgs.msg
import control_msgs.msg
#import message_filters

class GripperCommandAction(object):
    """Create a GripperCommandAction as a standard interface for the simulated gripper.

    This is pretty much just a wrapper to use both simulated finger controllers via one
    action simultaneously. The max_effort parameter of the command will be ignored as
    well as no stalling will be detected.

    Also, as moveit seems to use the joint_states_desired topic and expects.....TODO
    """
    _left_finger_index = -1
    _right_finger_index = -1
    _feedback = control_msgs.msg.GripperCommandFeedback()
    _result = control_msgs.msg.GripperCommandResult()

    def __init__(self, name):
        #self._left_state_subscriber = message_filters.Subscriber("franka_left_finger_controller/state")
        #self._right_state_subscriber = message_filters.Subscriber("franka_right_finger_controller/state")
        #self._approximate_synchronizer = message_filters.ApproximateTimeSynchronizer([self._left_state_subscriber, self._right_state_subscriber], 1)
        #self._approximate_synchronizer.registerCallback(self.state_cb)
        self._joint_state_subscriber = rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.state_cb, queue_size=1)
        self._command_publisher_left = rospy.Publisher("franka_left_finger_controller/command", std_msgs.msg.Float64, queue_size=1)
        self._command_publisher_right = rospy.Publisher("franka_right_finger_controller/command", std_msgs.msg.Float64, queue_size=1)
        self._action_name = name
        self._action_server = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._action_server.start()

        #self._joint_state_desired_publisher = rospy.Publisher("joint_states_desired", sensor_msgs.msg.JointState, queue_size=1)

    def state_cb(self, joint_state):
        # get indices where to find values for each finger in joint state message
        if self._left_finger_index == -1 or self._right_finger_index == -1:
            #TODO Better method than looking for hardcoded joint names?
            self._left_finger_index = joint_state.name.index('panda_finger_joint1')
            self._right_finger_index = joint_state.name.index('panda_finger_joint2')
        else:
            self._feedback.position = joint_state.position[self._left_finger_index] + joint_state.position[self._right_finger_index]
            self._feedback.effort = joint_state.effort[self._left_finger_index] + joint_state.effort[self._right_finger_index]

        # switch order of joint state message
        # if self._left_finger_index <= 1:
        #     joint_state.name = list(joint_state.name)
        #     joint_state.position = list(joint_state.position)
        #     joint_state.velocity = list(joint_state.velocity)
        #     joint_state.effort = list(joint_state.effort)
        #     for i in range (0, 2):
        #         joint_state.name.append(joint_state.name.pop(0))
        #         joint_state.position.append(joint_state.position.pop(0))
        #         joint_state.velocity.append(joint_state.velocity.pop(0))
        #         joint_state.effort.append(joint_state.effort.pop(0))
        #     self._joint_state_desired_publisher.publish(joint_state)

    def execute_cb(self, goal):
        # helper variables
        success = True

        # set feedback values
        #self._feedback.position = 0 #TODO
        #self._feedback.effort = 0 #TODO
        self._feedback.stalled = False
        self._feedback.reached_goal = False

        # start executing the action
        self._command_publisher_left.publish(std_msgs.msg.Float64(goal.command.position/2))
        self._command_publisher_right.publish(std_msgs.msg.Float64(goal.command.position/2))
        while self._feedback.reached_goal == False:
            # check that preempt has not been requested by the client
            if self._action_server.is_preempt_requested():
                self._action_server.set_preempted()
                success = False
                break

            #TODO update position???
            error = goal.command.position - self._feedback.position
            if abs(error) < 0.0014:
                self._feedback.reached_goal = True

            # publish the feedback
            self._action_server.publish_feedback(self._feedback)

        if success:
            self._result.position = self._feedback.position
            self._result.effort = self._feedback.effort
            self._result.stalled = self._feedback.stalled
            self._result.reached_goal = self._feedback.reached_goal
            self._action_server.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('franka_gripper')
    server = GripperCommandAction(rospy.get_name() + "/gripper_action")
    rospy.spin()
