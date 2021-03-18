#!/usr/bin/env python

from sensor_msgs.msg import CompressedImage
import numpy as np
import time
import rospy, math, cv2, os

global count
count=0
class slip_detection_reaction:

    def __init__(self):
 
        self.image_sub = rospy.Subscriber("/raspicam_node1/image/compressed",CompressedImage,self.call_back,queue_size = 1,buff_size=2**24)

    def rgb2gray(self,rgb):
        return np.dot(rgb[...,:3], [0.5, 0.5, 0.])

    def call_back(self,data):
        t = time.time()
        np_arr = np.fromstring(data.data, np.uint8)
        

        
        raw_imag = cv2.imdecode(np_arr, cv2.IMREAD_COLOR).astype(np.uint8)

        ab_array = np.load('abe_corr.npz')
        x_index = ab_array['x']
        y_index = ab_array['y']
        raw_imag = raw_imag[x_index, y_index, :]

        cv2.imshow('raw_image',raw_imag)
        #cv2.imshow('red',raw_imag[:,:,2])
        #cv2.imshow('green',raw_imag[:,:,1])
        #cv2.imshow('blue',raw_imag[:,:,0])
        cv2.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

def main():
    print("\n[INFO] Press Ctrl+C in terminal to exit camera live feed...")
    rospy.init_node('slip_detector', anonymous=True)
    while not rospy.is_shutdown():

    	slip_detector = slip_detection_reaction()
    	rospy.spin()


if __name__ == "__main__": 
    main()
    
#%%
    
    
