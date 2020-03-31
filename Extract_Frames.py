#============================================================
import numpy as np
import matplotlib.pyplot as plt 
import cv2 as cv
#============================================================
# Reading Videos
#--------------------
vid_1 = cv.VideoCapture('Belal_checkboard/1.mp4')
vid_2 = cv.VideoCapture('Belal_checkboard/2.mp4')
#============================================================
# Extracting Frames
#--------------------
ctr = 0
while(vid_1.isOpened()):
    ret1, frame1 = vid_1.read()
    if ret1 == False:
        break
    if ret1 == True:
        cv.imwrite('frame1/frame_1_%d.jpg'%ctr, frame1)
        ctr = ctr+1
#------------------------------------------------------------        
ctr = 0
while(vid_2.isOpened()):
    ret2, frame2 = vid_2.read()
    if ret2 == False:
        break
    if ret2 == True:
        cv.imwrite('frame2/frame_2_%d.jpg'%ctr, frame2)
        ctr = ctr+1
#============================================================
vid_1.release()
vid_2.release()
cv.destroyAllWindows()
#============================================================
