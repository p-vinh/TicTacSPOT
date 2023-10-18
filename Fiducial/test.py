import cv2 as cv
from cv2 import aruco
import numpy as np
from aprilTag import apriltag
from cv2 import FileStorage, FileNode

# Could use aprilTag library instead of ArUco
def main():
    # marker1 = cv.imread("Images\\523.jpg", cv.IMREAD_GRAYSCALE)
    
    # Create a custom ArUco dictionary
    # byteList = aruco.Dictionary.getByteListFromBits(np.array(str))
    # aruco_dict = aruco.Dictionary(np.array(str), 8, 6)
    
    # marker = {{0,0,0,0,0,0,0,0},{0,0,0,0,1,1,1,0},{0,0,0,1,1,1,0,0},{0,1,1,0,0,0,1,0},{0,0,1,0,0,1,0,0},{0,0,1,1,0,0,0,0},{0,0,0,1,0,0,1,0},{0,0,0,0,0,0,0,0}};

    
    # # Save the dictionary to a file
    # fs = cv.FileStorage("arucoDict.yaml", cv.FILE_STORAGE_WRITE)
    # fs.write("aruco_dict", np.array([marker1]))
    # fs.release()
    
    # Load the dictionary from a file
    # aruco_dict_loaded = aruco.Dictionary_load("arucoDict.yml")
    pass
main()