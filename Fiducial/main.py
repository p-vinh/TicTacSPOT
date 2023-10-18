import cv2 as cv
from cv2 import aruco
import numpy as np
from queue import Queue
from config_file_reader import process_config_file #This is a function in config_file_reader.py


general_settings, markers = process_config_file('tracker_config_file.ini')


def generateMarker():
    markerDict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    for i in range(20):
        marker = aruco.generateImageMarker(markerDict, i, 700)
        cv.imwrite(f"Images\\marker{i}.jpg", marker)
    
    
def detectFiducial():

    markerDict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        
    param_marker = aruco.DetectorParameters()
    cap = cv.VideoCapture(0)
    
    #################################
    # This is the camera matrix, assembled from calibration.
    camera_matrix = np.array( [[general_settings["camera_focal_length_x"], 0, general_settings["camera_center_x"]], [0, general_settings["camera_focal_length_y"], general_settings["camera_center_x"]], [0, 0, 1]], dtype = "double" )
    # Coefficients are [k1, k2, p1, p2, k3] as per OpenCV documentation
    distortion_coefficients = np.array( [general_settings["camera_k1"], general_settings["camera_k2"], general_settings["camera_p1"], general_settings["camera_p2"], general_settings["camera_k3"]] )
    #################################
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCandidates = aruco.detectMarkers(gray_frame, markerDict, parameters=param_marker)
        
        if markerCorners:
            frame = aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

                            
        cv.imshow("frame", frame)        
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv.destroyAllWindows()
    


def main():
    detectFiducial()
if __name__ == '__main__':
    main()