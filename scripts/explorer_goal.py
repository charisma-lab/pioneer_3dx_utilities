#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionFeedback
from nav_msgs.msg import OccupancyGrid
import numpy as np

import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from math import sqrt, atan2, sin, cos

state = 'stopped'

def movebase_client(goalPose):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = goalPose
    #goal.target_pose.pose.position.x = 0.0
    #goal.target_pose.pose.orientation.w = 1.0

    state= 'moving'
    client.send_goal(goal)
    wait = client.wait_for_result()
    state = 'stopped'
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        print(client.get_result())
        return client.get_result()

def resultCB(result_msg):
    print(result_msg.status.text)

def feedbackCB(fb_msg):
    return(fb_msg.feedback.base_position.pose)

def mapCB(map_msg):
    if(state == 'moving'):
        return
    origin = map_msg.info.origin.position
    w = map_msg.info.width # width in cells
    h = map_msg.info.height # height in cells
    res = map_msg.info.resolution # [m/cell]
    
    omap = np.asarray(map_msg.data, dtype=np.int8).reshape(h, w)

    # get the cell limits
    minC = max(toPix(minX - origin.x, res), 0)
    maxC = min(toPix(maxX - origin.x, res), w)
    minR = max(toPix(minY - origin.y, res), 0)
    maxR = min(toPix(maxY - origin.y, res), h)

    #print(f"({minC},{maxC}), ({minR},{maxR})")

    reducedMap = omap[minR:maxR, minC:maxC]

    # debug stuff
    debugMsg = OccupancyGrid()
    debugMsg.header.frame_id = map_msg.header.frame_id
    debugMsg.header.stamp = map_msg.header.stamp

    newX = toCoord(minC+1, res) + origin.x
    newY = toCoord(minR+1, res) + origin.y
    debugMsg.info.resolution = map_msg.info.resolution
    debugMsg.info.width = maxC - minC
    debugMsg.info.height = maxR - minR
    debugMsg.info.origin.position.x = toCoord(minC+1, res) + origin.x
    debugMsg.info.origin.position.y = toCoord(minR+1, res) + origin.y
    debugMsg.info.origin.orientation.w = 1.0

    

    # get robot position in map frame
    trans = tfBuffer.lookup_transform('base_link', 'map', rospy.Time())

    rPos = trans.transform.translation
    rLoc = (toPix(rPos.x - newX, res), toPix(rPos.y - newY, res))

    goalLoc = getGoalLocs(reducedMap, rLoc)

    # get goal in realpose
    goalPos = ( toCoord(goalLoc[1],res) + newX, toCoord(goalLoc[0],res) + newY )

    goal = Pose()
    goal.position.x = goalPos[0]
    goal.position.y = goalPos[1]

    theta = atan2( goalPos[1] - goal.position.y, goalPos[0] - goal.position.x)

    goal.orientation.z = sin(theta/2.0)

    goal.orientation.w = cos(theta/2.0)

    #print(goal)
    movebase_client(goal)

    #dbPub.publish(goal)

    
    #print(reducedMap.shape)
    #debugMsg.data = reducedMap.ravel()
    #print("Pubing")
    #mapPub.publish(debugMsg)

def freeNeighbor(mp, r, c):
    
    
    return mp[r,max(0,c-1)] == -1 or mp[r,min(c+1,mp.shape[1]-1)] == -1 or mp[max(0,r-1),c] == -1 or mp[min(r+1,mp.shape[0]-1),c]==-1
    
def getGoalLocs(mp, rLoc):
    maxDist = 0.0
    idxs = (0,0)
    count = 0
    for r in range(mp.shape[0]):
        for c in range(len(mp[0])):
            if(mp[r,c] >= 0 and mp[r,c] < 0.5 and not freeNeighbor(mp, r, c)):
                count += 1
                dx = rLoc[0] - c
                dy = rLoc[1] - r
                dist = sqrt(dx * dx + dy * dy)
                if(dist > maxDist):
                    maxDist = dist
                    idxs = (r,c)
    print(count)
    return(idxs)
                
    
def toPix(x, res):
    return int( x/res)

def toCoord(x, res):
    return (x * res) 

"""
minX = rospy.get_param('minX')
maxX = rospy.get_param('maxX')
minY = rospy.get_param('minY')
maxY = rospy.get_param('maxY')
"""
minX = -1.0
maxX = 5-1.0
minY = -1.0
maxY = 5.0-1.0

tfBuffer = None
listener = None

if __name__ == '__main__':
    rospy.init_node('movebase_client_py')

    mapSub = rospy.Subscriber('/map', OccupancyGrid, mapCB)    
    resSub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, resultCB)      
    fbSub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, feedbackCB)
    #mapPub = rospy.Publisher('mapDebug', OccupancyGrid, queue_size=5)
    #dbPub = rospy.Publisher('goalDebug', Pose, queue_size=5)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    #result = movebase_client()
    #if result:
    #    rospy.loginfo("Goal execution done!")
    rospy.spin()
