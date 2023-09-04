#!/usr/bin/env python


import cv2
import numpy as np
import rospy

# from st import stitch_frames
# ****************************************
# simplly show all(4)  frames get by video capture 
# ****************************************

def main():
    cap0 = cv2.VideoCapture('/dev/video0') 
    cap1 = cv2.VideoCapture('/dev/video2')
    cap2 = cv2.VideoCapture('/dev/video4')
    cap3 = cv2.VideoCapture('/dev/video6')

    while True:
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        ret3, frame3 = cap3.read()
        
        # be sure get the view success
        if not ret0 and not ret1 and not ret2 and not ret3:
            break

        # stitched_frame = np.hstack(( frame0, frame1, frame2, frame3 ))
        
        # show view
        # cv2.imshow('Webcam', stitched_frame)
        if ( ret0 ) :
            cv2.imshow('Webcam0', frame0)
        if ( ret1 ) :
            cv2.imshow('Webcam1', frame1)
        if ( ret2 ) :
            cv2.imshow('Webcam2', frame2)
        if ( ret3 ) :
            cv2.imshow('Webcam3', frame3)

        # press 'q' to end the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # release camera 
    cap0.release()
    cap1.release()
    cap2.release()
    cap3.release()

    # close all windows
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
