#!/usr/bin/python
import encodertest
import time

import numpy as np
import cv2
from rectifier import Rectifier
from orienter import Orienter

def main():

    # Initialize Vision Processors
    cv2.ocl.setUseOpenCL(False)
    cam = cv2.VideoCapture(0)
    rect = Rectifier('camera.yaml')
    ori = Orienter()
    cam_ok,img = cam.read()

    # Initialize Encoder
    enc = encodertest.encodertest()
    enc.toggle_led3()

    angle_enc = 0
    angle_cam = 0

    angle_encs = []
    angle_cams = []
    angle_diffs = []

    while cam_ok:
        angleBytes = enc.enc_readReg(enc.ENC_ANGLE_AFTER_ZERO_POS_ADDER)
        angle_enc = int(angleBytes[0])+int(angleBytes[1])*256
        angle_enc = (float(angle_enc) / 0x3FFF * 360) % 360

        cam_ok,img = cam.read()
        img = rect.apply(img)
        res,angle_cam,frames = ori.apply(img,draw=True)
        if res: 
            roi, roi_masked, proc, img = frames
            angle_encs.append(angle_enc)
            angle_cams.append(angle_cam)
            angle_diffs.append(angle_enc - angle_cam)
            cv2.imshow('Orientation', roi_masked)
        k = cv2.waitKey(20)
        if k == 27:
            break
        print 'enc : {}, cam : {}'.format(angle_enc, angle_cam)
    
    angle_encs, angle_cams, angle_diffs = [np.unwrap(np.deg2rad(x)) for x in (angle_encs, angle_cams, angle_diffs)]
    norm_data = np.c_[angle_encs, angle_cams, angle_diffs]

    np.savetxt('calib.csv', norm_data, header='angle_enc angle_cam diff')


if __name__ == "__main__":
    main()

#enc = encodertest.encodertest()
#
#enc.toggle_led3()
#
#while (True):
#    angleBytes = enc.enc_readReg(enc.ENC_ANGLE_AFTER_ZERO_POS_ADDER)
#
#    angle = int(angleBytes[0])+int(angleBytes[1])*256
#    print "Bin: {0:016b} Dec:{0:0d}".format(angle)
#
#    time.sleep(.02)
