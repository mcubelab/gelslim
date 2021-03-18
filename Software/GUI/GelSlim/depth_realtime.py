#!/usr/bin/env python

from sensor_msgs.msg import CompressedImage, JointState, ChannelFloat32
from std_msgs.msg import Bool
import numpy as np
import time
from scipy import ndimage
import matplotlib.pyplot as plt
from visualization_msgs.msg import *
import rospy, math, cv2, os, pickle
import std_srvs.srv
from fast_poisson import fast_poisson
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
import collections
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-c","--calibration", required=True, help="file path of calibration table")
args=vars(ap.parse_args())

class slip_detection_reaction:
    def __init__(self):

        self.img_counter1 = 0
        self.img_counter2 = 0
        self.table = np.load('{}'.format(args["calibration"]))#('table_gelsight2.npy')  #change this with your own table
        self.abe_array = np.load('abe_corr.npz') # change this with your aberration array 
        self.x_index = self.abe_array['x']
        self.y_index = self.abe_array['y']
        self.pad = 20
        self.zeropoint = -90
        self.lookscale = 180
        self.bin_num = 90
        self.con_flag1 = True
        self.con_flag2 = True
        self.reset_shape1 = True
        self.reset_shape2 = True
        self.restart1 = False
        self.restart2 = False
        self.reset_trigger1 = False
        self.reset_trigger2 = False
        self.slip_indicator1 = False
        self.slip_indicator2 = False
        self.refresh1 = False
        self.refresh2 = False
        self.showimage1 = False
        self.scale = 1
        self.kernel = self.make_kernel(31, 'circle')
        self.kernel2 = self.make_kernel(9, 'circle')
        self.mpq = collections.deque(maxlen=1800)

        self.image_sub2 = rospy.Subscriber("/raspicam_node1/image/compressed",
                                           CompressedImage,
                                           self.call_back1,
                                           queue_size=1,
                                           buff_size=2**24)

    def init_figure(self):
        self.fig = plt.figure(10, figsize=plt.figaspect(0.25))
        self.ax = self.fig.gca(projection='3d')
        self.ax.auto_scale_xyz([0, 427], [0, 320], [0, 1.5])
        self.ax.set_xlim3d(0, 427)
        self.ax.set_ylim3d(0, 320)
        self.ax.set_zlim3d(0, 1)
        self.sc = self.ax.scatter(0, 0, 0)
        # self.ax.set_aspect('equal')

        self.fig.show()

    def call_back1(self, data):
        t = time.time()
        if self.con_flag1:
            np_arr = np.fromstring(data.data, np.uint8)
            raw_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            raw_image = raw_image[self.x_index, self.y_index, :]
            ref_image = self.crop_image(raw_image, self.pad)
            marker = self.marker_detection(ref_image.copy())
            keypoints = self.find_dots((1 - marker) * 255)
            if self.reset_shape1:
                marker_mask = self.make_mask(ref_image.copy(), keypoints)
                ref_image = cv2.inpaint(ref_image, marker_mask, 3,
                                        cv2.INPAINT_TELEA)
                #self.red_mask = (ref_image > 12).astype(np.uint8)
                self.red_mask = (ref_image[:,:,2] > 12).astype(np.uint8)
                self.dmask1 = self.defect_mask(ref_image[:, :, 0])
                self.ref_blur1 = cv2.GaussianBlur(ref_image.astype(np.float32),
                                                  (3, 3), 0)
                self.blur_inverse1 = 1 + ((np.mean(self.ref_blur1) /
                                           (self.ref_blur1 + 1)) - 1) * 2
                # self.init_figure()
            ######### marker tracking##############
            # self.u_sum1 = np.zeros(len(keypoints))
            # self.v_sum1 = np.zeros(len(keypoints))
            self.u_addon1 = list(np.zeros(len(keypoints)))
            self.v_addon1 = list(np.zeros(len(keypoints)))
            self.x_iniref1 = []
            self.y_iniref1 = []
            marker_num = len(keypoints)
            self.mp_array = np.zeros((marker_num, 3, 200))
            for i in range(marker_num):
                self.x_iniref1.append(keypoints[i].pt[0] / self.scale)
                self.y_iniref1.append(keypoints[i].pt[1] / self.scale)
                self.mp_array[i, :, self.img_counter1 % 100] = np.array(
                    [keypoints[i].pt[0], keypoints[i].pt[1], 0])
            self.index_ref = np.linspace(0, marker_num - 1,
                                         marker_num).astype(int)
            self.con_flag1 = False
            self.reset_shape1 = False
        else:
            if self.restart1:
                self.con_flag1 = True
                self.restart1 = False

            np_arr = np.fromstring(data.data, np.uint8)
            raw_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            raw_image = raw_image[self.x_index, self.y_index, :]
            # cv2.imwrite('./data/' + 'img_' + str(self.img_counter1) + '.jpg',
            #             raw_image)
            # raw_image = cv2.imread('./test_data_2/img_209.jpg')
            raw_image = self.crop_image(raw_image, self.pad)
            raw_image = cv2.GaussianBlur(raw_image.astype(np.float32), (3, 3),
                                         0)

            marker_mask = self.marker_detection(raw_image) * self.dmask1
            grad_img2 = self.matching_v2(raw_image, self.ref_blur1,
                                         self.blur_inverse1)
            depth = fast_poisson(grad_img2[:, :, 0] * (1 - marker_mask) * self.red_mask,
                                 grad_img2[:, :, 1] * (1 - marker_mask) * self.red_mask)
            depth[depth < 1] = 0
            depth = cv2.applyColorMap((depth * (-100)).astype(np.uint8),
                                      cv2.COLORMAP_BONE)
            cv2.imshow('depth', depth.astype(np.uint8) * 255)
            cv2.waitKey(1)
            # print(np.max(depth))
            # contact_marker_mask = (depth > 0.1) * marker_mask
            # contact = self.contact_detection(raw_image, self.ref_blur1,
            #                                  marker_mask, self.kernel)
            # keypoints = self.find_dots((1 - marker_mask) * 255)
            # if self.img_counter1 < 2:
            #     self.marker_num = len(keypoints)
            # x1, y1, x2, y2, u, v, self.x_iniref1, self.y_iniref1, self.u_addon1, self.v_addon1\
            #     = self.flow_calculate_global(keypoints, list(self.x_iniref1), list(self.y_iniref1), \
            #         list(self.u_addon1), list(self.v_addon1), contact_marker_mask, depth)

            # keypoints = self.find_dots((1-contact_marker_mask)*255)
            # x_marker = np.array(y2).astype(int)
            # y_marker = np.array(x2).astype(int)
            # z_marker = depth[x_marker, y_marker]

            # if self.showimage1:
            #     self.dispOpticalFlow(raw_image, np.array(x2), np.array(y2),
            #                          np.array(u), np.array(v), 'flow1',
            #                          self.slip_indicator1)
            # contact_masks = depth > 0.1
            # print('length', np.mean(np.abs(u)+np.abs(v)), self.reset_trigger1)
            # print(x1.shape[0], self.marker_num)
            # if x1.shape[0] + 1 < self.marker_num and np.sum(
            #         contact_marker_mask) == 0:
            #     self.restart1 = True
            #     self.reset_trigger1 = False
            #     print('reset')

            # if np.mean(np.abs(u) + np.abs(v)) > 3:
            #     self.reset_trigger1 = True

            # print('xyz', self.mp_array[0,:,self.img_counter1%100])
            # print('shape', self.mp_array.shape)
            # plt.pause(0.001)
            # self.sc._offsets3d = (self.mp_array[:,0,self.img_counter1%100-50:self.img_counter1%100].flatten(), \
            #     self.mp_array[:,1,self.img_counter1%100-50:self.img_counter1%100].flatten(),\
            #  self.mp_array[:,2,self.img_counter1%100-50:self.img_counter1%100].flatten())
            # plt.draw()
            # cv2.imshow('raw_image',
            #            ((raw_image - self.ref_blur1) + 100).astype(np.uint8))
            # cv2.imshow('gradient',
            #            ((grad_img2[:, :, 0]) * 200).astype(np.uint8))
            # print('max', np.max(grad_img2[:, :, 0]), 'min',
            # np.min(grad_img2[:, :, 0]))
            # key = cv2.waitKey(1)
            # if key == 27:
            #     self.restart1 = True

        self.img_counter1 += 1
        #print(1 / (time.time() - t))

    def crop_image(self, img, pad):
        return img[pad:-pad, pad:-pad]

    def defect_mask(self, img):
        pad = 20
        var0 = 60  #left up
        var1 = 60  # right up
        var2 = 65  # right down
        var3 = 60  # left down
        im_mask = np.ones((img.shape))
        # triangle0 = np.array([[0, 0], [var0, 0], [0, var0]])
        # triangle1 = np.array([[im_mask.shape[1] - var1, 0],
        #                       [im_mask.shape[1], 0], [im_mask.shape[1], var1]])
        # triangle2 = np.array([[im_mask.shape[1] - var2, im_mask.shape[0]], [im_mask.shape[1], im_mask.shape[0]], \
        #     [im_mask.shape[1], im_mask.shape[0]-var2]])
        # triangle3 = np.array([[0, im_mask.shape[0]],
        #                       [0, im_mask.shape[0] - var3],
        #                       [var3, im_mask.shape[0]]])
        # color = [0]  #im_mask
        # cv2.fillConvexPoly(im_mask, triangle0, color)
        # cv2.fillConvexPoly(im_mask, triangle1, color)
        # cv2.fillConvexPoly(im_mask, triangle2, color)
        # cv2.fillConvexPoly(im_mask, triangle3, color)
        im_mask[:pad, :] = 0
        im_mask[-pad:, :] = 0
        im_mask[:, :pad * 2 + 20] = 0
        im_mask[:, -pad:] = 0
        return im_mask.astype(int)

    def flow_calculate_in_contact(self, keypoints2, x_initial, y_initial,
                                  u_ref, v_ref):
        x2, y2, u, v, x1_paired, y1_paired, x2_paired, y2_paired = [], [], [], [], [], [], [], []

        refresh = False
        for i in range(len(keypoints2)):
            x2.append(keypoints2[i].pt[0] / self.scale)
            y2.append(keypoints2[i].pt[1] / self.scale)

        x2 = np.array(x2)
        y2 = np.array(y2)

        for i in range(x2.shape[0]):

            distance = list(((np.array(x_initial) - x2[i])**2 +
                             (np.array(y_initial) - y2[i])**2))
            if len(distance) == 0:
                break
            min_index = distance.index(min(distance))
            u_temp = x2[i] - x_initial[min_index]
            v_temp = y2[i] - y_initial[min_index]
            shift_length = np.sqrt(u_temp**2 + v_temp**2)
            # print 'length',shift_length

            if shift_length < 12:
                # print xy2.shape,min_index,len(distance)
                x1_paired.append(x_initial[min_index] - u_ref[min_index])
                y1_paired.append(y_initial[min_index] - v_ref[min_index])
                x2_paired.append(x2[i])
                y2_paired.append(y2[i])
                u.append(u_temp + u_ref[min_index])
                v.append(v_temp + v_ref[min_index])

                del x_initial[min_index], y_initial[min_index], u_ref[
                    min_index], v_ref[min_index]

                if shift_length > 7:
                    refresh = True

        return x1_paired, y1_paired, x2_paired, y2_paired, u, v, refresh

    def flow_calculate_global(self, keypoints2, x_initial, y_initial, u_ref,
                              v_ref, contact_mask, depth):
        x2, y2, u, v, x1_paired, y1_paired, x2_paired, y2_paired  = [], [], [], [], [], [], [], []
        x1_return, y1_return, x2_return, y2_return, u_return, v_return = [],[],[],[],[],[]
        index_ref = []

        for i in range(len(keypoints2)):
            x2.append(keypoints2[i].pt[0] / self.scale)
            y2.append(keypoints2[i].pt[1] / self.scale)

        x2 = np.array(x2)
        y2 = np.array(y2)

        for i in range(x2.shape[0]):
            distance = (((np.array(x_initial) - x2[i])**2 +
                         (np.array(y_initial) - y2[i])**2))

            if len(distance) == 0:
                break
            min_index = np.argmin(distance)

            u_temp = x2[i] - x_initial[min_index]
            v_temp = y2[i] - y_initial[min_index]
            shift_length = np.sqrt(u_temp**2 + v_temp**2)
            # print 'length',shift_length
            if shift_length < 12:
                x1_paired.append(x_initial[min_index] - u_ref[min_index])
                y1_paired.append(y_initial[min_index] - v_ref[min_index])
                x2_paired.append(x2[i])
                y2_paired.append(y2[i])
                index_ref.append(self.index_ref[min_index])
                self.mp_array[self.index_ref[min_index],:,self.img_counter1%100] = \
                np.array([x2[i], y2[i], depth[int(y2[i]), int(x2[i])]])
                u.append(u_temp + u_ref[min_index])
                v.append(v_temp + v_ref[min_index])

                # del x_initial[min_index], y_initial[min_index], u_ref[
                #     min_index], v_ref[min_index]

        self.index_ref = list(index_ref)
        inbound_check = contact_mask[np.array(y2_paired).astype(np.uint16),
                                     np.array(x2_paired).
                                     astype(np.uint16)] * np.array(
                                         range(len(x2_paired))).astype(int)

        final_list = list(set(inbound_check) - set([0]))
        x1_inbound = np.array(x1_paired)[final_list]
        y1_inbound = np.array(y1_paired)[final_list]
        x2_inbound = np.array(x2_paired)[final_list]
        y2_inbound = np.array(y2_paired)[final_list]
        u_inbound = np.array(u)[final_list]
        v_inbound = np.array(v)[final_list]

        x1_return = np.array(x1_paired)
        y1_return = np.array(y1_paired)
        x2_return = np.array(x2_paired)
        y2_return = np.array(y2_paired)
        u_return = np.array(u)
        v_return = np.array(v)

        return x1_return, y1_return, x2_return, y2_return, u_return, v_return, \
            list(x2_paired), list(y2_paired), list(u), list(v)

    def matching_v2(self, test_img, ref_blur, blur_inverse):
        diff_temp1 = test_img - ref_blur
        diff_temp2 = diff_temp1 * blur_inverse
        diff_temp3 = np.clip((diff_temp2 - self.zeropoint) / self.lookscale, 0,
                             0.999)
        diff = (diff_temp3 * self.bin_num).astype(int)
        grad_img = self.table[diff[:, :, 0], diff[:, :, 1], diff[:, :, 2], :]
        return grad_img

    def find_dots(self, binary_image):
        # down_image = cv2.resize(binary_image, None, fx=2, fy=2)
        params = cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 1
        params.maxThreshold = 12
        params.minDistBetweenBlobs = 9
        params.filterByArea = True
        params.minArea = 5
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False
        params.minInertiaRatio = 0.5
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(binary_image.astype(np.uint8))

        return keypoints

    def make_mask(self, img, keypoints):
        img = np.zeros_like(img[:, :, 0])
        for i in range(len(keypoints)):
            # cv2.circle(img, (int(keypoints[i].pt[0]), int(keypoints[i].pt[1])), 6, (1), -1)
            cv2.ellipse(img,
                        (int(keypoints[i].pt[0]), int(keypoints[i].pt[1])),
                        (9, 6), 0, 0, 360, (1), -1)

        return img

    def marker_detection(self, raw_image):
        m, n = raw_image.shape[1], raw_image.shape[0]
        # raw_image = cv2.pyrDown(raw_image).astype(np.float32)
        raw_image_blur = cv2.GaussianBlur(raw_image.astype(np.float32), (5, 5),
                                          0)
        ref_blur = cv2.GaussianBlur(raw_image.astype(np.float32), (25, 25), 0)
        diff = ref_blur - raw_image_blur
        diff *= 16.0
        diff[diff < 0.] = 0.
        diff[diff > 255.] = 255.
        mask = ((diff[:, :, 0] > 25) & (diff[:, :, 2] > 25) &
                (diff[:, :, 1] > 120))
        mask = cv2.resize(mask.astype(np.uint8), (m, n))
        mask = cv2.dilate(mask, self.kernel2, iterations=1)
        return mask

    def contact_detection(self, raw_image, ref_blur, marker_mask, kernel):
        diff_img = np.max(np.abs(raw_image.astype(np.float32) - ref_blur),
                          axis=2)
        contact_mask = (((diff_img > 12).astype(np.uint8) *
                         (1 - marker_mask)) * self.dmask1).astype(np.uint8)
        print('max', np.max(diff_img))
        cv2.imshow('contact', contact_mask * 255)
        cv2.waitKey(1)
        contact_mask = cv2.dilate(contact_mask, kernel, iterations=1)
        contact_mask = cv2.erode(contact_mask, kernel, iterations=1)
        cv2.imshow('contact_mask', contact_mask * 255)
        cv2.waitKey(1)
        return contact_mask

    def make_kernel(self, n, k_type):
        if k_type == 'circle':
            kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))
        else:
            kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (n, n))
        return kernal

    def dispOpticalFlow(self, im_cal, x, y, u, v, name, slip_indicator):
        # mask = np.zeros_like(im_cal)
        mask2 = np.zeros_like(im_cal)
        amf = 1
        x = np.array(x).astype(np.int16)
        y = np.array(y).astype(np.int16)
        for i in range(u.shape[0]):  #self.u_sum

            mask2 = cv2.line(mask2,
                             (int(x[i] + u[i] * amf), int(y[i] + v[i] * amf)),
                             (x[i], y[i]), [0, 120, 120], 2)

        img = cv2.add(im_cal / 1.5, mask2)

        # if slip_indicator:
        #     img = img + self.im_slipsign / 2

        cv2.imshow(name, img.astype(np.uint8))
        cv2.waitKey(1)


def main():
    print "[INFO] Press Ctrl+C to exit and reutrn to GUI..."
    plt.ion()
    rospy.init_node('depth_reconstruction', anonymous=True)
    while not rospy.is_shutdown():
        # time.sleep(2)
        # open_gripper()
        # time.sleep(1)
        # force_initial = 10
        # close_gripper_f(50,force_initial)
        # time.sleep(0.1)
        slip_detector = slip_detection_reaction()
        rospy.spin()


if __name__ == "__main__":
    main()

#%%
