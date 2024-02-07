from math import sin
controlHeight = 0
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
