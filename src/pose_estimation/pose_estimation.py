import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

# Fake camera matrix and distortion coefficients for testing with webcam
# These are typical values for a standard webcam with ~640x480 resolution
camera_matrix = np.array([
    [650, 0, 320],
    [0, 650, 240],
    [0, 0, 1]
], dtype=np.float32)

# Distortion coefficients (k1, k2, p1, p2, k3)
dist_coeffs = np.array([0.1, -0.2, 0.001, 0.001, 0.0], dtype=np.float32)

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


def arucoDetect(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    aruco_dict = cv.aruco.Dictionary_get(ARUCO_DICT["DICT_5X5_1000"])

    parameters = cv.aruco.DetectorParameters_create()

    corners, ids, _ = cv. aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

    if ids is not None:
        cv.putText(frame, str(ids), (int(corners[0][0][0][0]), int(corners[0][0][0][1])), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
    
    return frame


def pose_estimation(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    aruco_dict = cv.aruco.Dictionary_get(ARUCO_DICT["DICT_5X5_1000"])

    parameters = cv.aruco.DetectorParameters_create()

    corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

    if ids is not None and len(ids) > 0 and len(corners) > 0:
        try:
            # Tegn marker IDs
            cv.putText(frame, str(ids[0]), (int(corners[0][0][0][0]), int(corners[0][0][0][1])), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
            
            # Estimér pose
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
            
            # Tegn akserne for hver markør
            for i in range(len(ids)):
                cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
        except Exception as e:
            print(f"Error in pose_estimation: {e}")
            import traceback
            traceback.print_exc()
    
    return frame

cam = cv.VideoCapture(0)

while cam.isOpened():
    ret, frame = cam.read()

    try:
        frame = pose_estimation(frame)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        break

    cv.imshow('frame', frame )

    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cam.release()
cv.destroyAllWindows()