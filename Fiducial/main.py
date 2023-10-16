import logging
import math
import signal
import sys
import threading
import time
from sys import platform

import cv2
import numpy as np
from PIL import Image

        
def showCamera():
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Camera", frame)
        
        # Detect fiducial. Get Corners
        img = cv.imread(gray_frame)
        gray= cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        
        sift = cv.SIFT_create()
        kp = sift.detect(gray,None)
        img=cv.drawKeypoints(gray,kp,img)
        cv.imwrite('test\sift_keypoints.jpg',img)
        
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
    

def main():
    showCamera()
    
    

if __name__ == '__main__':
    main()