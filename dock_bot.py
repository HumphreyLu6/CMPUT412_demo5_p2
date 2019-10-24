#!/usr/bin/env python
import numpy as np
import rospy, actionlib
import time
import cv2, cv_bridge
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import smach
import smach_ros

def goal_pose(pose): 
    goal_pose = MoveBaseGoal()

    goal_pose.target_pose.header.frame_id = 'target'

    goal_pose.target_pose.pose.position.x = pose.position.x - 0.05
    goal_pose.target_pose.pose.position.y = pose.position.y
    goal_pose.target_pose.pose.position.z = pose.position.z
    
    goal_pose.target_pose.pose.orientation.x = pose.orientation.x
    goal_pose.target_pose.pose.orientation.y = pose.orientation.y
    goal_pose.target_pose.pose.orientation.z = pose.orientation.z
    goal_pose.target_pose.pose.orientation.w = pose.orientation.w

    return goal_pose

class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['found','search','end'])
        self.prevDockARTags = []
        print "dasd"

    def execute(self, userdata):
        rospy.loginfo('State: Search')

        if rospy.is_shutdown():
            return 'end'
            
        found = False
        found = self.searchDock()
        if found:
            return 'found'
        else:
            return 'search'

    def searchDock(self):
        global g_tags, g_target_pose
        found_new_tag = False
        if len(g_tags) == 0:
            return False
        for tag in g_tags:
            if tag not in self.prevDockARTags:
                self.prevDockARTags.append(tag)
                g_target_pose = g_tags[tag]
                found_new_tag = True
                return True

        if found_new_tag == False:
            return False

class Dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['endDock','end'])
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.client.wait_for_server()
    
    def execute(self, userdata):
        global g_target_pose
        rospy.loginfo('State: Dock')
        if rospy.is_shutdown():
            return 'end'

        goal = goal_pose(g_target_pose)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return 'endDock'

class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['return', 'search','end'])
    
    def execute(self, userdata):
        rospy.loginfo('State: Return')
        if rospy.is_shutdown():
            return 'end'

        isBack = False
        if isBack:
            return 'search'
        else:
            return 'return'

def ar_callback(msg):
    global g_tags
    for marker in msg.markers:
        g_tags[int(marker.id)] = marker.pose.pose

def main():
    rospy.init_node("dock_bot")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback)

    sm = smach.StateMachine(outcomes=['end'])

    with sm:
        smach.StateMachine.add('Search', Search(),
                                transitions={'search':'Search',
                                             'found':'Dock',
                                             'end':'end'})

        smach.StateMachine.add('Dock', Dock(),
                                transitions={'endDock':'Return',
                                             'end':'end'})

        smach.StateMachine.add('Return', Return(),
                                transitions={'return':'Return',
                                             'search':'Search',
                                             'end':'end'})
    outcome = sm.execute()
    
    rospy.spin()

    if outcome == 'end':
        cv2.destroyAllWindows()

if __name__ == "__main__":
    g_tags = {}
    g_target_pose = None
    main()   