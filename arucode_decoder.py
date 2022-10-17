#!/usr/bin/env python
# https://github.com/KhairulIzwan/ArUco-markers-with-OpenCV-and-Python/blob/main/Detecting%20ArUco%20markers/detect_aruco_video.py

# import the necessary packages
from typing import Any

from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys
import numpy as np

class Arucode_Decoder():
    def __init__(self):
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11) #cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
        self.arucoParams = cv2.aruco.DetectorParameters_create()

        imsize = (1024, 768)
        self.cameraMatrix = cv2.getDefaultNewCameraMatrix(np.diag([1024, 768, 1]), imsize, True)
        self.distortionCoeffs = np.ndarray([0])
        self.marker_length = 50

    def decode(self,frame):
        Line_Pts=None
        Dist = [None,None]
        # loop over the frames from the video stream

        frame = imutils.resize(frame, width=1000)

        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,self.arucoDict, parameters=self.arucoParams)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()

            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the axis
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.marker_length, self.cameraMatrix, distCoeffs=None)
                print(rvec)
                print(tvec)
                cv2.drawFrameAxes(frame, self.cameraMatrix, None, rvec, tvec, 10, 3)

                #rmat, jacobian = cv2.Rodrigues(rvec)
                #print(rmat)
                points_to_daw_3Dcoords = np.float32([[10,10,0],[20, 0, 0], [0, 20, 0], [0, 0, 20]]).reshape(-1, 3)
                points_to_daw_2Dcoords, jacobian = cv2.projectPoints(points_to_daw_3Dcoords, rvec, tvec, self.cameraMatrix, self.qdistortionCoeffs)
                for p in points_to_daw_2Dcoords:
                    cv2.circle(frame, (int(p[0][0]),int(p[0][1])), 2, (0,124,124), 20)

                # draw the bounding box of the ArUCo detection
                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (255, 255, 0), 2)

                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

                # draw the ArUco marker ID on the frame
                cv2.putText(frame, "{}: {} ".format(str(markerID),"Landing Pad"),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        return frame
