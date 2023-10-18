import cv2 as cv
from cv2 import aruco
import numpy as np
from cv2 import FileStorage, FileNode

def main():
    marker1 = cv.imread("Images\\523.jpg", cv.IMREAD_GRAYSCALE)
    print(np.array([marker1]))
    # Create a custom ArUco dictionary
    aruco_dict = aruco.Dictionary(np.array([marker1]), 8, 6)
    
    # Save the dictionary to a file
    fs = cv.FileStorage("arucoDict.yaml", cv.FILE_STORAGE_WRITE)
    fs.write("aruco_dict", np.array([marker1]))
    fs.release()
    
    # Load the dictionary from a file
    # aruco_dict_loaded = aruco.Dictionary_load("arucoDict.yml")
main()