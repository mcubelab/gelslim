#!/usr/bin/env python

from sensor_msgs.msg import CompressedImage
import numpy as np
import time
import rospy, math, cv2, os, sys
global count;
import tkSimpleDialog as simpledialog
from Tkinter import *
import Tkinter , Tkconstants, tkFileDialog
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-i","--images",    required=True, help="Number of calibration images to caputre")
args=vars(ap.parse_args())
global count;
count=0;

print('[INFO] Capturing '+'{}'.format(args["images"])+' images...')
num=int('{}'.format(args["images"]))
print('[INFO] Do not press the calibration ball to the sensor for the first image (ref.jpg)...')
print('[INFO] '+'{}'.format(args["images"])+' images will be captured at a regular time interval...')
print('[INFO] Beginning calibration image capture...')
time.sleep(2)
class slip_detection_reaction:

    def __init__(self):
 
        self.image_sub = rospy.Subscriber("/raspicam_node1/image/compressed",CompressedImage,self.call_back,queue_size = 1,buff_size=2**24)

    def rgb2gray(self,rgb):
        return np.dot(rgb[...,:3], [0.5, 0.5, 0.])

    def call_back(self,data):
        t = time.time()
        np_arr = np.fromstring(data.data, np.uint8)
        raw_imag = cv2.imdecode(np_arr, cv2.IMREAD_COLOR).astype(np.uint8)

        cv2.imshow('raw_image',raw_imag)
        #cv2.imshow('red',raw_imag[:,:,2])
        #cv2.imshow('green',raw_imag[:,:,1])
        #cv2.imshow('blue',raw_imag[:,:,0])
        #time.sleep(2)
        cv2.waitKey(1)

        global count
        global end    
        count+=1
        if count %100 == 0 and count/100==1:
            cv2.imwrite('./test_data/ref.jpg',raw_imag)
            print('[INFO] Saved reference image to ~/test_data')
        if count %100 == 0 and count/100>1 and count/100<=(num+1):
            cv2.imwrite(('./test_data/sample_'+str((count/100)-1)+'.jpg'),raw_imag)
            print('[INFO] Saved sample '+str((count/100)-1)+' to ~/test_data')
        if count/100 >=(num+1):
            cv2.destroyAllWindows()
            print('[INFO] Calibration image capture complete...')
            print('[INFO] Please remove empty images from ./test_data')
            print('[INFO] Press Ctrl+C in terminal to exit camera and return to GUI...')
            time.sleep(100)
        else:
            exit(0)

def main():
    rospy.init_node('slip_detector', anonymous=True)
    while not rospy.is_shutdown():

    	slip_detector = slip_detection_reaction()
    	rospy.spin()




if __name__ == "__main__": 
    main()
  
#%%
  
    
