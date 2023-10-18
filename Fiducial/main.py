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
        sift = cv2.SIFT_create()
        
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        kp = sift.detect(gray_frame,None)
        img = cv2.drawKeypoints(gray_frame,kp,gray_frame)
        cv2.imshow("SIFT", img)        
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
    

def main():
    showCamera()
    
    

if __name__ == '__main__':
    main()