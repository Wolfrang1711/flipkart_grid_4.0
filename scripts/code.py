#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Pose2D
import numpy as np
from cv2 import aruco
import math
cap = cv2.VideoCapture(-1)
bridge = CvBridge()
global aruco_msg
x_centre=0
y_centre =0
def talker():
    aruco_publisher = rospy.Publisher('detected_aruco', Pose2D, queue_size=10)
    rospy.init_node("Image", anonymous = False)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        current_frame = cv2.resize(frame, (500, 500), interpolation=cv2.INTER_LINEAR)

        marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        param_markers = cv2.aruco.DetectorParameters_create()
		
        marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(current_frame, marker_dict, parameters=param_markers)

        if marker_corners:
            for ids, corners in zip(marker_IDs, marker_corners):
            
                cv2.polylines(current_frame, [corners.astype(np.int32)], True, (0, 0, 255), 4, cv2.LINE_AA)
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[1].ravel()
                cv2.putText(current_frame,f"id: {ids[0]}", top_right, cv2.FONT_HERSHEY_PLAIN, 1.3, (200, 100, 0), 2, cv2.LINE_AA,)
                x_centre = int((corners[1][0] + corners[3][0])/2)
                y_centre = int((corners[1][1] + corners[3][1])/2)
                centre = (x_centre, y_centre)
                cv2.circle(current_frame, centre, 2, (0, 0, 255), -1)
                x_horizontal = int((corners[1][0] + corners[2][0])/2)
                y_horizontal = int((corners[1][1] + corners[2][1])/2)

                horizontal = (x_horizontal, y_horizontal)

                cv2.line(current_frame, centre, horizontal, (0, 255, 0), 2)

                inv_m1 = math.atan2((y_horizontal - y_centre),
                                    (x_horizontal - x_centre))
            
                aruco_msg.x = x_centre
                aruco_msg.y = y_centre        
                print(x_centre,y_centre,inv_m1)

        aruco_publisher.publish(aruco_msg)
        cv2.imshow("frame", current_frame)
        cv2.waitKey(10)
        if rospy.is_shutdown():
            cap.release()

if __name__ == "__main__":
    aruco_msg= Pose2D()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
