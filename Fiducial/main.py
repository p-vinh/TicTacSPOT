import cv2 as cv
from cv2 import aruco
import numpy as np
from queue import Queue
from cv2 import FileStorage, FileNode
from config_file_reader import process_config_file #This is a function in config_file_reader.py


general_settings, markers = process_config_file('tracker_config_file.ini')


def customDictionary():
    marker1 = cv.imread("Images\\523.jpg", cv.IMREAD_GRAYSCALE)
    print(np.array([marker1]))
    # Create a custom ArUco dictionary
    aruco_dict = aruco.Dictionary(np.array([marker1]), 8, 6)
    
    # Save the dictionary to a file
    fs = cv.FileStorage("arucoDict.yaml", cv.FILE_STORAGE_WRITE)
    fs.write("aruco_dict", np.array([marker1]))
    fs.release()
 



def generateMarker():
    markerDict = aruco.getPredefinedDictionary(aruco.DICT_7X7_1000)
    for i in range(10):
        marker = aruco.generateImageMarker(markerDict, i, 700)
        cv.imwrite(f"Images\\marker{i}.jpg", marker)
    
    
def showCamera():
    fs = cv.FileStorage("arucoDict.yaml", cv.FILE_STORAGE_READ)
    bytesList = np.frombuffer(fs.getNode("aruco_dict").mat().tobytes(), dtype=np.uint8)
    nMarkers = fs.getNode("nMarkers").real()
    markerSize = fs.getNode("markerSize").real()
    markerDict = aruco.Dictionary_getByteListFromBits(bytesList)
    fs.release()
    
    # for item in markerDict.writeDictionary():
    #     print(item)
    
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
    showCamera()
if __name__ == '__main__':
    main()