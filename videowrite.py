
import re
import matplotlib
matplotlib.use("Agg")
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import sys
import glob
from IPython import display
import os

#used from OpenCV Tuturial
numbers = re.compile(r"(\d+)")
def numericalSort(value):
    parts = numbers.split(value)
    parts[1::2] = map(int, parts[1::2])
    return parts

c = 0
img_array = []
for filename in sorted(glob.glob(r"C:/Users/Adam/Desktop/661/images/*.jpg"), key=numericalSort):
    img = cv.imread(filename)
    img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)

#output video
out = cv.VideoWriter('results_final.avi',cv.VideoWriter_fourcc(*'DIVX'), 15, size)

for i in range(len(img_array)):
    rgb_img = cv.cvtColor(img_array[i], cv.COLOR_RGB2BGR)
    out.write(rgb_img)
out.release()