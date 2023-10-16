import logging
import math
import signal
import sys
import threading
import time
from sys import platform

import cv2
from cv2 import aruco
import PySimpleGUI as sg
import numpy as np
from PIL import Image


class DetectFiducial:
    def __init__(self):
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()

    def detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        return corners, ids, rejected

def showCamera():
    cap = cv2.VideoCapture(0)
    detector = DetectFiducial()
    
    while True:
        ret, frame = cap.read()
        corners, ids, rejected = detector.detect(frame)
       # aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("Camera", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
    

def main():
    test = DetectFiducial()
    showCamera()


if __name__ == '__main__':
    main()