import rclpy
from rclpy.node import Node
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

#For let løsninger laves det første bare som opencv, image-information skal ændres til at bruge real-sense Node
#Men det er letter at løse pt ved brug af normal kamera.

 


#just to generate a aruco marker, we will use a video caputed of aruco marker
ARUCO_DICT = {
    "DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
    "DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
    "DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
    "DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv.aruco.DICT_APRILTAG_36h11
    }

def deteced_aruco_marker(frame):
    aruco_type_list = []

    for aruco_type, dict_id in ARUCO_DICT.items():
        arucoDict = cv.aruco.getPredefinedDictionary(dict_id)
        arucoParams = cv.aruco.DetectorParameters()


        corners, ids, _ = cv. aruco.detectMarkers(frame, arucoDict, parameters = arucoParams)

        if len (corners) > 0:
            aruco_type_list.append(aruco_type)

            print(f"Markers deceted using {aruco_type} dictionary")

            for markerCorner, markerId in zip(corners, ids.flatten()):
                corners_aruco = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners_aruco

                cv.polylines(frame, [markerCorner.astype(int)], True, (0, 255, 0), 2)

                cX = int((topLeft[0] + bottomRight[0]) / 2)
                cY = int((topLeft[1] + bottomRight[1]) / 2)

                cv.circle(frame, (cX, cY), 5, (255, 0, 0), -1)
                cv.putText(frame, str(aruco_type) + " " + str(int(markerId)),
                            (int(topLeft[0] - 5), int(topLeft[1])), cv.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 255))
                
    return aruco_type_list


def pose_estimation(frame, aruco_dict_type,):

    aruco_dict = cv.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv.aruco.DetectorParameters()

    corners, ids = cv.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if len(corners) > 0:
        for i in range(0, len(ids)):
        
            rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners[i], 0.025)
            
            cv.aruco.drawDetectedMarkers(frame, corners) 

            cv.drawFrameAxes(frame, rvec, tvec, 0.01) 
            
    return frame




cam = cv.VideoCapture(0)

while cam.isOpened:
    ret, frame = cam.read()

    for aruco_type in deteced_aruco_marker(frame):
        frame = pose_estimation(frame, ARUCO_DICT[aruco_type])

    cv.imshow('frame', frame )

    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cam.release()
cv.destroyAllWindows()