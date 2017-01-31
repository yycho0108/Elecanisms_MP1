#!/usr/bin/python
import cv2
import numpy as np
import rospkg
import os

from yaml import load, Loader

def to_matrix(data):
    rows = data['rows']
    cols = data['cols']
    return np.reshape(data['data'], (rows,cols)).astype(np.float32)

def get_matrices(file_path):
    with open(file_path) as stream:
        data = load(stream, Loader=Loader)
        m = to_matrix(data['camera_matrix'])
        d = to_matrix(data['distortion_coefficients'])
        r = to_matrix(data['rectification_matrix'])
        p = to_matrix(data['projection_matrix'])
        return m, d, r, p

class Rectifier(object):
    def __init__(self, param):
        m, d, r, p = get_matrices(param)
        self.m1, self.m2 = cv2.initUndistortRectifyMap(m,d,r,p, (848,480), cv2.CV_32FC1)
    def apply(self, img):
        proc = cv2.remap(img, self.m1, self.m2, cv2.INTER_LINEAR)
        return proc

if __name__ == "__main__":
    cam = cv2.VideoCapture(0)

    rect = Rectifier('camera.yaml')
    ret = True
    while ret:
        ret,img = cam.read()
        im_rect = rect.apply(img)
        cv2.imshow("im_raw", img)
        cv2.imshow("im_rect", im_rect)

        # Verification?
        im_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        im_rect_gray = cv2.cvtColor(im_rect, cv2.COLOR_BGR2GRAY)
        diff = cv2.absdiff(im_rect_gray, im_gray)
        cv2.imshow("diff",diff)
        if cv2.waitKey(20) == 27:
            break
