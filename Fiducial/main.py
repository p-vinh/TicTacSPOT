import cv2 as cv
from cv2 import aruco
import numpy as np

        
def showCamera():
    markerDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    param_marker = aruco.DetectorParameters_create()
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCandidates = aruco.detectMarkers(gray_frame, markerDict, parameters=param_marker)
        
        if markerCorners:
            for ids, corners in zip(markerIds, markerCorners):
                cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 0), 4, cv.LINE_AA)
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                botton_left = corners[3].ravel()
                
                cv.putText(frame, f"id: {ids[0]}", top_right, cv.FONT_HERSHEY_SIMPLEX, 1.3, (200, 100, 0), 2, cv.LINE_AA)
                
                
                
        cv2.imshow("frame", frame)        
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
    

def main():
    showCamera()    

if __name__ == '__main__':
    main()