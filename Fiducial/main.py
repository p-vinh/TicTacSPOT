import cv2 as cv
from cv2 import aruco
import numpy as np

def generateMarker():
    markerDict = aruco.getPredefinedDictionary(aruco.DICT_7X7_1000)
    for i in range(10):
        marker = aruco.generateImageMarker(markerDict, i, 700)
        cv.imwrite(f"Images\\marker{i}.jpg", marker)
        
def showCamera():
    markerDict = aruco.getPredefinedDictionary(aruco.DICT_7X7_1000)
    param_marker = aruco.DetectorParameters()
    cap = cv.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCandidates = aruco.detectMarkers(gray_frame, markerDict, parameters=param_marker)
        
        print("Marker Ids: ", markerIds)
        print("\nMarker Corners: ", markerCorners)
        
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
                
                
                
        cv.imshow("frame", frame)        
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv.destroyAllWindows()
    

def main():
    generateMarker()      

if __name__ == '__main__':
    main()