import cv2 as cv
from cv2 import aruco
import numpy as np
import time
from queue import Queue

from config_file_reader import process_config_file #This is a function in config_file_reader.py

import sys
import tictactoe as ttt
import boardInput as bi

general_settings, markers = process_config_file('C:\\Users\\laure\\OneDrive\\Documents\\GitHub\\TicTacSPOT\\Fiducial\\tracker_config_file.ini')

# DICT_APRILTAG_16h5 ---> small
# DICT_APRILTAG_25h9 ---> medium
# DICT_APRILTAG_36h11 ---> large

def get_input():
    """
    Get input from the user.
    """
    while True:
        try:
            x = int(input("Enter a number: "))
            break
        except ValueError:
            print("Invalid input. Try again.")
    return x

#------------------------------------------Generate Marker main function-----------------------------------------------------------
    
def generateMarker():
    markerDict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    for i in range(20):
        marker = aruco.generateImageMarker(markerDict, i, 700)
        cv.imwrite(f"Images\\marker{i}.jpg", marker)
    


#------------------------------------------Detect Fiducial main function----------------------------------------------------------- 
def detectFiducial():
    markerDict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        
    param_marker = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(markerDict, param_marker)
    cap = cv.VideoCapture(0)
    
    #################################
    # This is the camera matrix, assembled from calibration.
    camera_matrix = np.array( [[general_settings["camera_focal_length_x"], 0, general_settings["camera_center_x"]], [0, general_settings["camera_focal_length_y"], general_settings["camera_center_x"]], [0, 0, 1]], dtype = "double" )
    # Coefficients are [k1, k2, p1, p2, k3] as per OpenCV documentation
    distortion_coefficients = np.array( [general_settings["camera_k1"], general_settings["camera_k2"], general_settings["camera_p1"], general_settings["camera_p2"], general_settings["camera_k3"]] )
    #################################
    
    start_time = time.time()
    interval = 3
    detectArray = [] 
    player = ttt.X  
    while True:
        # Detect     
        ret, frame = cap.read()
                
        if not ret:
            break
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray_frame)
        
        if markerCorners:
            frame = aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
            
            elapsed_time = time.time() - start_time
            
            # Update Board
            if elapsed_time > interval:
                detectArray = convertTo2DArray(markerIds)
                print("Input:")
                print(detectArray)
                bi.updateTotalPieces(bi)
                valid = bi.checkValidInput(detectArray)
                
                if (valid):
                    if (player == ttt.X): #Players Turn                   
                        bi.updateBoard(detectArray,player,bi)
                        print("Player is")
                        print(player)
                        bi.totalXPieces += 1
                if (player == ttt.O): # AI Turn        
                    print("AI Move: ")
                    move = ttt.minimax(bi.boardState)
                    print(move)
                    bi.boardState = ttt.result(bi.boardState, move)

                    bi.totalOPieces += 1
                displayBoard()
                
                if ttt.terminal(bi.boardState):
                    print("Winner is")
                    print(ttt.winner(bi.boardState))
                    break

                
                
                player = ttt.player(bi.boardState)  
                start_time = time.time()
            
            
        cv.imshow("frame", frame)  
              
        

        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv.destroyAllWindows()
   
def displayBoard():
    for i in range(3):
        for j in range(3):
            print(bi.boardState[i][j], end=" ")
        print()
 
def convertTo2DArray(markerIds):
    ret = []
    for i in range(len(markerIds)):
        ret.append(markerIds[i][0])
    return ret

    
if __name__ == '__main__':
    detectFiducial()