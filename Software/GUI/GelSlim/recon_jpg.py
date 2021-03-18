#!/usr/bin/env python


from sensor_msgs.msg import CompressedImage
import numpy as np
import time
import rospy, math, cv2, os

global count;
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-i","--images",    required=True, help="Number of calibration images to caputre")
args=vars(ap.parse_args())
print('[INFO] Capturing '+'{}'.format(args["images"])+' images...')
num=int('{}'.format(args["images"]))
print('[INFO] '+'{}'.format(args["images"])+' images will be captured at a regular time interval...')
print('[INFO] Beginning reconstruction image capture...')
time.sleep(2)
count=0;

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
        cv2.waitKey(1)
		
	global count	
	count+=1
        #if count %100 == 0 and count/100==1:
            #cv2.imwrite(os.path.join(os.path.expanduser('~'),'gelsight_heightmap_reconstruction-master/python_version/test_data','ref.jpg'),raw_imag)
            #print('Saved reference image to ~/test_data')
        if count %100 == 0 and count/100>=1 and count/100<=(num+1):
            cv2.imwrite(('./reconstruction/images/'+'reconstruct_'+str((count/100)-1)+'.jpg'),raw_imag)
            print('Saved sample '+str((count/100)-1)+' to ./reconstruction/images')
        else:
            exit(0)

        #  if not cv2.imwrite(os.path.join(os.path.expanduser('~'),'/home/ian/Documents/gelsight_heightmap_reconstruction-master/python_version/test_data','ref.jpg'),raw_imag):
           # raise Exception("could not write image")    

def main():
    print "[INFO] Press Ctrl+C to exit and return to GUI..."
    rospy.init_node('slip_detector', anonymous=True)
    while not rospy.is_shutdown():

    	slip_detector = slip_detection_reaction()
    	rospy.spin()


if __name__ == "__main__": 
    main()
  
#%%
  
    
