#!/usr/bin/env python
import numpy as np
import rospy, actionlib
import time
import cv2, cv_bridge
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
    import smach

class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['found', 'search','end'])
        self.prevDockARTags = []

    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'

        found = False
        found = self.searchDock()
        if found:
            return 'found'
        else:
            return 'search'

    def searchDock(self):
        global g_tagId
        tagId = g_tagId
        if tagId not in self.prevDockARTags:
            self.prevDockARTags.append(tagId)
            return True
        else:
            return False

class Dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['dock', 'endDock','end'])
    
    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'

        dockFinished = False
        if dockFinished:
            return 'endDock'
        else:
            return 'dock'


class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['return', 'search','end'])
    
    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'

        isBack = False
        if isBack:
            return 'search'
        else:
            return 'return'

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
                                transitions={'dock':'Dock',
                                             'endDock':'Return',
                                             'end':'end'})

        smach.StateMachine.add('Return', Return(),
                                transitions={'return':'Return',
                                             'search':'Search',
                                             'end':'end'})
    outcome = sm.execute()

    if outcome == 'end':
        cv2.destroyAllWindows()

    rospy.spin()

if __name__ == "__main__":
    g_tagId = None
    main()   