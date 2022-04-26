#!/usr/bin/env python3

import rospy
from actions.srv import StartActionRequest, StartAction, StartActionResponse

def start_action_callback(req):
    print("Got json: " + str(req.action_description_json))

    return StartActionResponse()


if __name__ == "__main__":
    rospy.init_node("actions_main", anonymous=True)
    rospy.loginfo("starting actions service listener")

    sa_serv = rospy.Service("/start_action", StartAction, start_action_callback)

    rospy.spin()
