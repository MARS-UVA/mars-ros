#!/usr/bin/env python3

import action_definitions
from actions.srv import StartActionRequest, StartAction, StartActionResponse
import json
import rospy
from threading import Thread


def validate_json_format(d):
    return ("name" in d and "update_delay" in d)

def start_action_in_background(action):
    rospy.loginfo("Starting action name=%s" % action.description["name"])
    t = Thread(target=action.start, args=())
    t.start()
    # Don't want to join here because we don't wait to wait for it to finish executing.

def start_action_callback(req):
    data = json.loads(req.action_description_json)
    print("Got json: " + str(data))

    if not validate_json_format(data):
        rospy.logwarn("Received action that was not formatted correctly!")

    name = data["name"]
    if name == "raise_bin":
        start_action_in_background(action_definitions.ActionRaiseBin(data))
    elif name == "lower_bin":
        start_action_in_background(action_definitions.ActionLowerBin(data))
    elif name == "raise_ladder":
        start_action_in_background(action_definitions.ActionRaiseLadder(data))
    elif name == "lower_ladder":
        start_action_in_background(action_definitions.ActionLowerLadder(data))
    elif name == "dig":
        rospy.loginfo("Starting ActionDig")
        start_action_in_background(action_definitions.ActionDig(data))
    elif name == "dump":
        rospy.loginfo("Starting ActionDump")
        start_action_in_background(action_definitions.ActionDump(data))
    else:
        rospy.logwarn("Received action name that was not recognized! name=%s" % name)

    return StartActionResponse()


if __name__ == "__main__":
    rospy.init_node("actions_node", anonymous=True)
    rospy.loginfo("starting actions service listener")

    sa_serv = rospy.Service("/start_action", StartAction, start_action_callback)

    rospy.spin()
