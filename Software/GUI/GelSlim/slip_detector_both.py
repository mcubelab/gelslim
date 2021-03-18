#!/usr/bin/env python

from sensor_msgs.msg import CompressedImage, JointState, ChannelFloat32
from std_msgs.msg import Bool
import numpy as np
import time
from scipy import ndimage
import matplotlib.pyplot as plt
from visualization_msgs.msg import *
from collections import deque
# from gripper import *
# from ik.helper import *
# from robot_comm.srv import *
# from wsg_50_common.msg import Status
import rospy, math, cv2, os, pickle
import std_srvs.srv
import time

timestr = time.strftime("%Y-%m-%d-%H%M%S")
#cap = cv2.VideoCapture(-1)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(('./videos/slip_'+timestr+'.avi'),fourcc,30.0,(427,320))

class slip_detector:
    def __init__(self):
        self.kernal = self.make_kernal(5, 'circle')
        self.kernal1 = self.make_kernal(4, 'rect')
        self.kernal2 = self.make_kernal(7, 'circle')
        self.kernal3 = self.make_kernal(25, 'circle')
        self.kernal4 = self.make_kernal(3, 'circle')
        self.kernal5 = self.make_kernal(5, 'rect')
        self.kernal6 = self.make_kernal(25, 'circle')
        self.kernal_size = 25
        self.kernal7 = self.make_kernal(self.kernal_size, 'circle')
        self.kernal8 = self.make_kernal(2, 'rect')
        self.kernal9 = self.make_kernal(2, 'rect')
        self.kernal10 = self.make_kernal(45, 'circle')
        self.cols, self.rows, self.cha = 320, 427, 3
        self.scale = 1
        self.con_flag1 = False
        self.con_flag2 = False
        self.refresh1 = False
        self.refresh2 = False
        self.restart1 = False
        self.restart2 = False
        self.slip_indicator1 = False
        self.slip_indicator2 = False
        self.abe_array = np.load('abe_corr.npz') # change this with your aberration array 
        self.x_index = self.abe_array['x']
        self.y_index = self.abe_array['y']
        self.image_sub1 = rospy.Subscriber("/raspicam_node1/image/compressed",
                                           CompressedImage,
                                           self.call_back1,
                                           queue_size=1,
                                           buff_size=2**24)
        self.image_sub2 = rospy.Subscriber("/raspicam_node2/image/compressed",
                                           CompressedImage,
                                           self.call_back2,
                                           queue_size=1,
                                           buff_size=2**24)

        self.collideThre = 2.5
        self.collide_rotation = 2.
        self.marker_thre = 100
        self.showimage1 = True
        self.showimage2 = False
        self.data1 = deque(maxlen=75)
        self.data2 = deque(maxlen=75)
        self.collision_detected = False
        self.calibrate1 = False
        self.calibrate2 = False
        self.counter1 = 0
        self.counter2 = 0
        # self.timestamp1 = deque(maxlen=75)
        # self.markermotion1 = deque(maxlen=75)
        # self.image2save2 = deque(maxlen=75)
        # self.timestamp2 = deque(maxlen=75)
        # self.markermotion2 = deque(maxlen=75)

    def rgb2gray(self, rgb):
        return np.dot(rgb[..., :3], [0.33, 0.33, 0.34])

    def make_kernal(self, n, type):
        if type is 'circle':
            kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))
        else:
            kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (n, n))
        return kernal

    def defect_mask(self, im_cal):
        pad = 1
        var0 = 60  #left up
        var1 = 60  # right up
        var2 = 65  # right down
        var3 = 60  # left down
        im_mask = np.ones((im_cal.shape))
        triangle0 = np.array([[0, 0], [var0, 0], [0, var0]])
        triangle1 = np.array([[im_mask.shape[1] - var1, 0],
                              [im_mask.shape[1], 0], [im_mask.shape[1], var1]])
        triangle2 = np.array([[im_mask.shape[1] - var2, im_mask.shape[0]], [im_mask.shape[1], im_mask.shape[0]], \
            [im_mask.shape[1], im_mask.shape[0]-var2]])
        triangle3 = np.array([[0, im_mask.shape[0]],
                              [0, im_mask.shape[0] - var3],
                              [var3, im_mask.shape[0]]])
        color = [0]  #im_mask
        cv2.fillConvexPoly(im_mask, triangle0, color)
        cv2.fillConvexPoly(im_mask, triangle1, color)
        cv2.fillConvexPoly(im_mask, triangle2, color)
        cv2.fillConvexPoly(im_mask, triangle3, color)
        im_mask[:pad, :] = 0
        im_mask[-pad:, :] = 0
        im_mask[:, :pad] = 0
        im_mask[:, -pad:] = 0
        return im_mask

    def make_thre_mask(self, im_cal):
        thre_image = np.zeros(im_cal.shape, dtype=np.uint8)
        previous_mask = np.zeros(im_cal.shape, dtype=np.uint8)
        for i in range(10, 80, 30):
            _, mask = cv2.threshold(im_cal.astype(np.uint8), i, 255,
                                    cv2.THRESH_BINARY_INV)
            mask_expand = cv2.dilate(mask, self.kernal10, iterations=1)
            mask_erode = cv2.erode(mask_expand, self.kernal10, iterations=1)
            thre_image += (mask_erode - previous_mask) / 255 * i
            previous_mask = mask_erode
            # cv2.imshow('threshold', thre_image)
            # cv2.waitKey(0)
        thre_image += (np.ones(im_cal.shape, dtype=np.uint8) -
                       previous_mask / 255) * 80 + 10

        return thre_image

    def creat_mask_2(self, raw_image, dmask):
        # t = time.time()
        scale = 2
        m, n = raw_image.shape[1], raw_image.shape[0]
        raw_image = cv2.pyrDown(raw_image).astype(np.float32)
        blur = cv2.GaussianBlur(raw_image, (15, 15), 0)
        blur2 = cv2.GaussianBlur(raw_image, (3, 3), 0)
        # print(time.time() - t)
        diff = blur - blur2
        # diff = cv2.resize(diff, (int(m / scale), int(n / scale)))
        # diff = (diff - np.min(diff)) / (np.max(diff) - np.min(diff)) * 255
        diff *= 15.0
        # cv2.imshow('blur2', blur.astype(np.uint8))
        # cv2.waitKey(1)

        diff[diff < 0.] = 0.
        diff[diff > 255.] = 255.

        # diff = cv2.GaussianBlur(diff, (5, 5), 0)
        # cv2.imshow('diff', diff.astype(np.uint8))
        # cv2.waitKey(1)
        mask_b = diff[:, :, 0] > 150
        mask_g = diff[:, :, 1] > 150
        mask_r = diff[:, :, 2] > 150

        mask = ((mask_b * mask_g) + (mask_b * mask_r) + (mask_g * mask_r)) > 0
        # cv2.imshow('mask', mask.astype(np.uint8) * 255)
        # cv2.waitKey(1)
        mask = cv2.resize(mask.astype(np.uint8), (m, n))
        mask = mask * dmask
        mask = cv2.dilate(mask, self.kernal4, iterations=1)
        # cv2.imshow('diff', diff.astype(np.uint8))
        # cv2.imshow('mask', mask.astype(np.uint8) * 255)
        # cv2.waitKey(1)
        # mask = cv2.erode(mask, self.kernal4, iterations=1)
        # print(time.time() - t)
        return (1 - mask) * 255

    def find_dots(self, binary_image):
        # down_image = cv2.resize(binary_image, None, fx=2, fy=2)
        params = cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 1
        params.maxThreshold = 12
        params.minDistBetweenBlobs = 9
        params.filterByArea = True
        params.minArea = 9
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False
        params.minInertiaRatio = 0.5
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(binary_image.astype(np.uint8))
        return keypoints

    def flow_calculate_in_contact_1(self, keypoints2):

        x2, y2, u, v, x1_paired, y1_paired, x2_paired, y2_paired = [], [], [], [], [], [], [], []
        temp_x = list(self.x_iniref1)
        temp_y = list(self.y_iniref1)
        temp_u = list(self.u_addon1)
        temp_v = list(self.v_addon1)

        for i in range(len(keypoints2)):
            x2.append(keypoints2[i].pt[0] / self.scale)
            y2.append(keypoints2[i].pt[1] / self.scale)

        x2 = np.array(x2)
        y2 = np.array(y2)
        refresh = False

        for i in range(x2.shape[0]):

            distance = list(((np.array(temp_x) - x2[i])**2 +
                             (np.array(temp_y) - y2[i])**2))
            if len(distance) == 0:
                break
            min_index = distance.index(min(distance))
            u_temp = x2[i] - temp_x[min_index]
            v_temp = y2[i] - temp_y[min_index]
            shift_length = np.sqrt(u_temp**2 + v_temp**2)
            # print 'length',shift_length

            if shift_length < 12:
                # print xy2.shape,min_index,len(distance)
                x1_paired.append(temp_x[min_index] - temp_u[min_index])
                y1_paired.append(temp_y[min_index] - temp_v[min_index])
                x2_paired.append(x2[i])
                y2_paired.append(y2[i])
                u.append(u_temp + temp_u[min_index])
                v.append(v_temp + temp_v[min_index])

                del temp_x[min_index], temp_y[min_index], temp_u[
                    min_index], temp_v[min_index]

                if shift_length > 10:
                    refresh = True

        return x1_paired, y1_paired, x2_paired, y2_paired, u, v, refresh

    def flow_calculate_in_contact_2(self, keypoints2):

        x2, y2, u, v, x1_paired, y1_paired, x2_paired, y2_paired = [], [], [], [], [], [], [], []
        temp_x = list(self.x_iniref2)
        temp_y = list(self.y_iniref2)
        temp_u = list(self.u_addon2)
        temp_v = list(self.v_addon2)

        for i in range(len(keypoints2)):
            x2.append(keypoints2[i].pt[0] / self.scale)
            y2.append(keypoints2[i].pt[1] / self.scale)

        x2 = np.array(x2)
        y2 = np.array(y2)
        refresh = False

        for i in range(x2.shape[0]):

            distance = list(((np.array(temp_x) - x2[i])**2 +
                             (np.array(temp_y) - y2[i])**2))
            if len(distance) == 0:
                break
            min_index = distance.index(min(distance))
            u_temp = x2[i] - temp_x[min_index]
            v_temp = y2[i] - temp_y[min_index]
            shift_length = np.sqrt(u_temp**2 + v_temp**2)
            # print 'length',shift_length

            if shift_length < 12:
                # print xy2.shape,min_index,len(distance)
                x1_paired.append(temp_x[min_index] - temp_u[min_index])
                y1_paired.append(temp_y[min_index] - temp_v[min_index])
                x2_paired.append(x2[i])
                y2_paired.append(y2[i])
                u.append(u_temp + temp_u[min_index])
                v.append(v_temp + temp_v[min_index])

                del temp_x[min_index], temp_y[min_index], temp_u[
                    min_index], temp_v[min_index]

                if shift_length > 10:
                    refresh = True

        return x1_paired, y1_paired, x2_paired, y2_paired, u, v, refresh

    def flow_calculate_global_1(self, keypoints2):

        x2, y2, u, v, x1_paired, y1_paired, x2_paired, y2_paired  = [], [], [], [], [], [], [], []
        x1_return, y1_return, x2_return, y2_return, u_return, v_return = [],[],[],[],[],[]

        for i in range(len(keypoints2)):
            x2.append(keypoints2[i].pt[0] / self.scale)
            y2.append(keypoints2[i].pt[1] / self.scale)

        x2 = np.array(x2)
        y2 = np.array(y2)

        for i in range(x2.shape[0]):
            distance = list(((np.array(self.x_iniref1) - x2[i])**2 +
                             (np.array(self.y_iniref1) - y2[i])**2))
            if len(distance) == 0:
                break
            min_index = distance.index(min(distance))
            u_temp = x2[i] - self.x_iniref1[min_index]
            v_temp = y2[i] - self.y_iniref1[min_index]
            shift_length = np.sqrt(u_temp**2 + v_temp**2)
            # print 'length',shift_length
            if shift_length < 12:
                x1_paired.append(self.x_iniref1[min_index] -
                                 self.u_addon1[min_index])
                y1_paired.append(self.y_iniref1[min_index] -
                                 self.v_addon1[min_index])
                x2_paired.append(x2[i])
                y2_paired.append(y2[i])
                u.append(u_temp + self.u_addon1[min_index])
                v.append(v_temp + self.v_addon1[min_index])

                del self.x_iniref1[min_index], self.y_iniref1[
                    min_index], self.u_addon1[min_index], self.v_addon1[
                        min_index]

        x1_return = np.array(x1_paired)
        y1_return = np.array(y1_paired)
        x2_return = np.array(x2_paired)
        y2_return = np.array(y2_paired)
        u_return = np.array(u)
        v_return = np.array(v)
        self.x_iniref1 = list(x2_paired)
        self.y_iniref1 = list(y2_paired)
        self.u_addon1 = list(u)
        self.v_addon1 = list(v)

        return x1_return, y1_return, x2_return, y2_return, u_return, v_return
        # return x1_paired,y1_paired,x2_paired,y2_paired,u,v

    def flow_calculate_global_2(self, keypoints2):

        x2, y2, u, v, x1_paired, y1_paired, x2_paired, y2_paired  = [], [], [], [], [], [], [], []
        x1_return, y1_return, x2_return, y2_return, u_return, v_return = [],[],[],[],[],[]

        for i in range(len(keypoints2)):
            x2.append(keypoints2[i].pt[0] / self.scale)
            y2.append(keypoints2[i].pt[1] / self.scale)

        x2 = np.array(x2)
        y2 = np.array(y2)

        for i in range(x2.shape[0]):
            distance = list(((np.array(self.x_iniref2) - x2[i])**2 +
                             (np.array(self.y_iniref2) - y2[i])**2))
            if len(distance) == 0:
                break
            min_index = distance.index(min(distance))
            u_temp = x2[i] - self.x_iniref2[min_index]
            v_temp = y2[i] - self.y_iniref2[min_index]
            shift_length = np.sqrt(u_temp**2 + v_temp**2)
            # print 'length',shift_length
            if shift_length < 12:
                x1_paired.append(self.x_iniref2[min_index] -
                                 self.u_addon2[min_index])
                y1_paired.append(self.y_iniref2[min_index] -
                                 self.v_addon2[min_index])
                x2_paired.append(x2[i])
                y2_paired.append(y2[i])
                u.append(u_temp + self.u_addon2[min_index])
                v.append(v_temp + self.v_addon2[min_index])

                del self.x_iniref2[min_index], self.y_iniref2[
                    min_index], self.u_addon2[min_index], self.v_addon2[
                        min_index]

        x1_return = np.array(x1_paired)
        y1_return = np.array(y1_paired)
        x2_return = np.array(x2_paired)
        y2_return = np.array(y2_paired)
        u_return = np.array(u)
        v_return = np.array(v)
        self.x_iniref2 = list(x2_paired)
        self.y_iniref2 = list(y2_paired)
        self.u_addon2 = list(u)
        self.v_addon2 = list(v)

        return x1_return, y1_return, x2_return, y2_return, u_return, v_return

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

        if slip_indicator:
            img = img + self.im_slipsign / 2
        out.write(img.astype(np.uint8))
        cv2.imshow(name, img.astype(np.uint8))
        cv2.waitKey(1)

        # raw_input("Press Enter to continue...")

    def call_back1(self, data):

        t = time.time()
        np_arr = np.fromstring(data.data, np.uint8)
        raw_imag = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        raw_imag = raw_imag[self.x_index, self.y_index, :]
        self.counter1 += 1
        self.counter1 *= (self.counter1 < 1e10)

        if not self.con_flag1:

            imgwc = np.array(raw_imag).astype(np.float32)
            self.im_slipsign = np.zeros(imgwc.shape)
            cv2.putText(self.im_slipsign, 'Slip', (210, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)
            im_gray = self.rgb2gray(imgwc)  #.astype(np.uint8)
            self.dmask1 = self.defect_mask(im_gray)
            final_image = self.creat_mask_2(imgwc, self.dmask1)
            keypoints = self.find_dots(final_image)
            self.u_sum1 = np.zeros(len(keypoints))
            self.v_sum1 = np.zeros(len(keypoints))
            self.u_addon1 = list(self.u_sum1)
            self.v_addon1 = list(self.v_sum1)
            self.x1_last1 = []
            self.y1_last1 = []
            for i in range(len(keypoints)):
                self.x1_last1.append(keypoints[i].pt[0] / self.scale)
                self.y1_last1.append(keypoints[i].pt[1] / self.scale)
            self.x_iniref1 = list(self.x1_last1)
            self.y_iniref1 = list(self.y1_last1)
            self.con_flag1 = True
            self.absmotion1 = 0
            # print("sensor 1 finishes pre-calculation")

        else:  #start detecting slip
            # print('time', time.time())
            imgwc = np.array(raw_imag).astype(np.float32)
            if self.calibrate1:
                if self.counter1 % 25 == 0:
                    cv2.imwrite(
                        './test_data/gelslim1_'
                        + str(self.counter1) + '.jpg', imgwc.astype(np.uint8))

            im_cal_show = np.array(imgwc)
            if self.restart1:
                self.con_flag1 = False
                self.restart1 = False
                self.absmotion1 = 0
                self.slip_indicator1 = False

            else:
                final_image = self.creat_mask_2(imgwc, self.dmask1)

                if self.refresh1:
                    keypoints = self.find_dots(final_image)
                    x1, y1, x2, y2, u, v = self.flow_calculate_global_1(
                        keypoints)
                    self.refresh1 = False
                else:
                    keypoints = self.find_dots(final_image)
                    x1, y1, x2, y2, u, v, self.refresh1 = self.flow_calculate_in_contact_1(
                        keypoints)

                # x2_center = np.expand_dims(np.array(x2), axis=1)
                # y2_center = np.expand_dims(np.array(y2), axis=1)
                # x1_center = np.expand_dims(np.array(x1), axis=1)
                # y1_center = np.expand_dims(np.array(y1), axis=1)
                # p2_center = np.expand_dims(np.concatenate(
                #     (x2_center, y2_center), axis=1),
                #                            axis=0)
                # p1_center = np.expand_dims(np.concatenate(
                #     (x1_center, y1_center), axis=1),
                #                            axis=0)
                # tran_matrix = cv2.estimateRigidTransform(
                #     p1_center, p2_center, False)
                # theta = np.arctan(
                #     -tran_matrix[0, 1] / tran_matrix[0, 0]) * 180. * np.pi
                theta = 0

                u_sum, v_sum, uv_sum = np.array(u), np.array(v), np.sqrt(
                    np.array(u)**2 + np.array(v)**2)
                self.absmotion1 = np.mean(uv_sum)

                # print('max',
                #       max(np.abs(np.mean(u_sum)), np.abs(np.mean(v_sum)),
                #           np.abs(np.mean(uv_sum))))

                if max(np.abs(np.mean(u_sum)), np.abs(np.mean(v_sum)),
                       np.abs(np.mean(uv_sum))) > 0.5:
                    self.collision_detected = True

                self.slip_indicator1 = max(
                    np.abs(np.mean(u_sum)), np.abs(np.mean(v_sum)),
                    np.abs(np.mean(uv_sum))) > self.collideThre or abs(
                        theta) > self.collide_rotation

                if self.showimage1:
                    self.dispOpticalFlow(im_cal_show, x2, y2, u_sum, v_sum,
                                         'flow1', self.slip_indicator1)

                # if self.slip_indicator1:
                #     print("sensor 1 slip!!!")

        self.data1.append([raw_imag, time.time(), self.absmotion1])

    def call_back2(self, data):
        # return

        t = time.time()
        np_arr = np.fromstring(data.data, np.uint8)
        raw_imag = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        raw_imag = raw_imag[self.x_index, self.y_index, :]
        self.counter2 += 1
        self.counter2 *= (self.counter2 < 1e10)

        if not self.con_flag2:

            imgwc = np.array(raw_imag).astype(np.float32)
            self.im_slipsign = np.zeros(imgwc.shape)
            cv2.putText(self.im_slipsign, 'Slip', (210, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)
            im_gray = self.rgb2gray(imgwc)  #.astype(np.uint8)
            self.dmask2 = self.defect_mask(im_gray)
            final_image = self.creat_mask_2(imgwc, self.dmask2)
            keypoints = self.find_dots(final_image)
            self.u_sum2 = np.zeros(len(keypoints))
            self.v_sum2 = np.zeros(len(keypoints))
            self.u_addon2 = list(self.u_sum2)
            self.v_addon2 = list(self.v_sum2)
            self.x1_last2 = []
            self.y1_last2 = []
            for i in range(len(keypoints)):
                self.x1_last2.append(keypoints[i].pt[0] / self.scale)
                self.y1_last2.append(keypoints[i].pt[1] / self.scale)
            self.x_iniref2 = list(self.x1_last2)
            self.y_iniref2 = list(self.y1_last2)
            self.con_flag2 = True
            self.absmotion2 = 0
            # print("sensor 2 finishes pre-calculation")

        else:  #start detecting slip
            # print('time', time.time())
            imgwc = np.array(raw_imag).astype(np.float32)
            if self.calibrate2:
                if self.counter2 % 1 == 0:
                    cv2.imwrite(
                        './test_data/gelslim2_'
                        + str(self.counter2) + '.jpg', imgwc.astype(np.uint8))

            im_cal_show = np.array(imgwc)

            if self.restart2:
                self.con_flag2 = False
                self.restart2 = False
                self.absmotion2 = 0
                self.slip_indicator2 = False

            else:
                final_image = self.creat_mask_2(imgwc, self.dmask2)

                if self.refresh2:
                    keypoints = self.find_dots(final_image)
                    x1, y1, x2, y2, u, v = self.flow_calculate_global_2(
                        keypoints)
                    self.refresh2 = False
                else:
                    keypoints = self.find_dots(final_image)

                    x1, y1, x2, y2, u, v, refresh = self.flow_calculate_in_contact_2(
                        keypoints)

                    self.refresh2 = refresh

                # x2_center = np.expand_dims(np.array(x2), axis=1)
                # y2_center = np.expand_dims(np.array(y2), axis=1)
                # x1_center = np.expand_dims(np.array(x1), axis=1)
                # y1_center = np.expand_dims(np.array(y1), axis=1)
                # p2_center = np.expand_dims(np.concatenate(
                #     (x2_center, y2_center), axis=1),
                #                            axis=0)
                # p1_center = np.expand_dims(np.concatenate(
                #     (x1_center, y1_center), axis=1),
                #                            axis=0)
                # tran_matrix = cv2.estimateRigidTransform(
                #     p1_center, p2_center, False)
                # if tran_matrix is not None:
                #     theta = np.arctan(
                #         -tran_matrix[0, 1] / tran_matrix[0, 0]) * 180. * np.pi
                # else:
                #     theta = 0

                theta = 0

                u_sum, v_sum, uv_sum = np.array(u), np.array(v), np.sqrt(
                    np.array(u)**2 + np.array(v)**2)
                self.absmotion2 = np.mean(uv_sum)

                if max(np.abs(np.mean(u_sum)), np.abs(np.mean(v_sum)),
                       np.abs(np.mean(uv_sum))) > 0.5:
                    self.collision_detected = True

                self.slip_indicator2 = max(
                    np.abs(np.mean(u_sum)), np.abs(np.mean(v_sum)),
                    np.abs(np.mean(uv_sum))) > self.collideThre or abs(
                        theta) > self.collide_rotation

                if self.showimage2:
                    self.dispOpticalFlow(im_cal_show, x2, y2, u_sum, v_sum,
                                         'flow2', self.slip_indicator2)

                # if self.slip_indicator2:
                #     print("sensor 2 slip!!!")

        # print('frequency', 1 / (time.time() - t))
        # self.data2.append([raw_imag, time.time(), self.absmotion2])


def main():
    #print "start"
    rospy.init_node('slip_detector', anonymous=True)
    while not rospy.is_shutdown():
        # time.sleep(2)
        # open_gripper()
        # time.sleep(1)
        # force_initial = 10
        # close_gripper_f(50,force_initial)
        # time.sleep(0.1)
        slip = slip_detector()
        rospy.spin()


if __name__ == "__main__":
    main()

#%%
