from math import sin
import rospy
from actions.srv import *
controlHeight = 0

LOWER_FLOOR = "lower_floor"
RAISE_FLOOR = "raise_floor"
DIG = "dig"

def onStart():
    global controlHeight
    controlHeight = getAvgHeight()

def getAvgHeight():
    readings = getReadings()
    totalHeight = 0.
    for (dist, angle) in readings:
        totalHeight += dist * sin(angle)
    avgHeight = totalHeight / len(readings)
    return avgHeight



'''if __name__ == "__main__":
	rospy.init_node("ir_computation")
	startActionClient = rospy.ServiceProxy('/start_action', StartAction)
	
	while not rospy.is_shutdown():
		req = f'{ "name":"{action}", "
'''
    

