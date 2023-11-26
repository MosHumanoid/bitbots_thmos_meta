#!/usr/bin/env python3

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from bitbots_msgs.msg import DynUpGoal, DynUpAction, DynUpFeedback

showing_feedback = False

if __name__ == "__main__":
    directions = ['front', 'back']
    direction_test = directions[0]

    rospy.init_node('dynup_client', anonymous=True)
    print("Initializing node")


    def done_cb(state, result):
        print('Action completed: ', end='')
        if state == GoalStatus.PENDING:
            print('Pending')
        elif state == GoalStatus.ACTIVE:
            print('Active')
        elif state == GoalStatus.PREEMPTED:
            print('Preempted')
        elif state == GoalStatus.SUCCEEDED:
            print('Succeeded')
        elif state == GoalStatus.ABORTED:
            print('Aborted')
        elif state == GoalStatus.REJECTED:
            print('Rejected')
        elif state == GoalStatus.PREEMPTING:
            print('Preempting')
        elif state == GoalStatus.RECALLING:
            print('Recalling')
        elif state == GoalStatus.RECALLED:
            print('Recalled')
        elif state == GoalStatus.LOST:
            print('Lost')
        else:
            print('Unknown state', state)
        print(str(result))


    def active_cb():
        print("Server accepted action")


    def feedback_cb(feedback):
        return 

    print('[..] Connecting to action server \'dynup\'', end='')
    client = actionlib.SimpleActionClient('thmos_dynup', DynUpAction)
    if not client.wait_for_server():
        exit(1)
    print('\r[OK] Connecting to action server \'dynup\'')
    print()

    goal = DynUpGoal()
    goal.direction = direction_test

    client.send_goal(goal)
    client.done_cb = done_cb
    client.feedback_cb = feedback_cb
    client.active_cb = active_cb
    print("Sent new goal. Waiting for result")
    client.wait_for_result()
