import cv2
from PIL import Image


def main():
    img = cv2.imread('cat_logo.jpg')
    print(img.shape)
    
main()