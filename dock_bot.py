#!/usr/bin/env python
import numpy as np
import rospy, actionlib
import time
import cv2, cv_bridge
from sensor_msgs.msg import Image
import smach

class Draw(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['draw', 'end'])
    
    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        else:
            self.draw()
            return 'draw'

    def draw(self):
        global g_image

        mtx = np.array([[522.102158,   0.000000, 314.103105],
                        [  0.000000, 524.312959, 260.228945],
                        [  0.000000,   0.000000,   1.000000]])  
        
        dist = np.array([-0.015797, -0.000331, -0.002507, -0.007520, 0.000000])

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((6*8,3), np.float32)
        objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

        axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

        while not rospy.is_shutdown():
            if g_image == None:
                print("No image")
                continue
            img = g_image

            gray = cv2.cvtColor(g_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (8,6), None)
            if ret == True:
                print(g_image.shape)
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

                # Find the rotation and translation vectors.
                _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

                # project 3D points to image plane
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

                img = draw(g_image,corners2,imgpts)
            cv2.imshow('img',img)
            cv2.waitKey(1)

def image_callback(msg):
    global g_image
    bridge = cv_bridge.CvBridge()
    g_image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
    #print(type(g_image))

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

def main():
    global g_image
    rospy.init_node("calibration_check")
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    sm = smach.StateMachine(outcomes=['end'])

    with sm:
        
        smach.StateMachine.add('Draw', Draw(),
                                transitions={'draw':'Draw',
                                             'end':'end'})
    outcome = sm.execute()

    if outcome == 'end':
        cv2.destroyAllWindows()

    rospy.spin()

if __name__ == "__main__":
    g_image = None
    main()   