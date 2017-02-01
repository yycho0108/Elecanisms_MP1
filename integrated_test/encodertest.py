#!/usr/bin/python
import numpy as np
import usb.core
import time

def parse_angle(angleBytes):
    angle_enc = int(angleBytes[0])+int(angleBytes[1])*256
    #print "Bin: {0:016b} Hex:{1:04x} Dec:{2:0f} Angle:{3:0f}".format(angle_enc, angle_enc, float(angle_enc)/0x3FFF, float(angle_enc)/0x3FFF*360)
    angle_enc = (float(angle_enc) / 0x3FFF * 360) % 360
    return angle_enc

class encodertest:

    def __init__(self):
        self.TOGGLE_LED1 = 1
        self.TOGGLE_LED2 = 2
        self.READ_SW1 = 3
        self.ENC_READ_REG = 5
        self.ENC_CLEAR_ERR = 6
        self.TOGGLE_LED3 = 8
        self.READ_SW2 = 9
        self.READ_SW3 = 10
        self.ENC_SET_ZERO=11
        self.dev = usb.core.find(idVendor = 0x6666, idProduct = 0x0003)
        if self.dev is None:
            raise ValueError('no USB device found matching idVendor = 0x6666 and idProduct = 0x0003')
        self.dev.set_configuration()

# AS5048A Register Map
        self.ENC_NOP = 0x0000
        self.ENC_CLEAR_ERROR_FLAG = 0x0001
        self.ENC_PROGRAMMING_CTRL = 0x0003
        self.ENC_OTP_ZERO_POS_HI = 0x0016
        self.ENC_OTP_ZERO_POS_LO = 0x0017
        self.ENC_DIAG_AND_AUTO_GAIN_CTRL = 0x3FFD
        self.ENC_MAGNITUDE = 0x3FFE
        self.ENC_ANGLE_AFTER_ZERO_POS_ADDER = 0x3FFF

    def close(self):
        self.dev = None

    def toggle_led1(self):
        try:
            self.dev.ctrl_transfer(0x40, self.TOGGLE_LED1)
        except usb.core.USBError:
            print "Could not send TOGGLE_LED1 vendor request."

    def toggle_led2(self):
        try:
            self.dev.ctrl_transfer(0x40, self.TOGGLE_LED2)
        except usb.core.USBError:
            print "Could not send TOGGLE_LED2 vendor request."

    def toggle_led3(self):
        try:
            self.dev.ctrl_transfer(0x40, self.TOGGLE_LED3)
        except usb.core.USBError:
            print "Could not send TOGGLE_LED3 vendor request."

    def read_sw1(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.READ_SW1, 0, 0, 1)
        except usb.core.USBError:
            print "Could not send READ_SW1 vendor request."
        else:
            return int(ret[0])

    def read_sw2(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.READ_SW2, 0, 0, 1)
        except usb.core.USBError:
            print "Could not send READ_SW2 vendor request."
        else:
            return int(ret[0])

    def read_sw3(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.READ_SW3, 0, 0, 1)
        except usb.core.USBError:
            print "Could not send READ_SW3 vendor request."
        else:
            return int(ret[0])

    def enc_setZero(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.ENC_SET_ZERO, 0, 0, 2)
        except usb.core.USBError:
            print "Could not send ENC_SET_ZERO vendor request."
        else:
            return ret

    def enc_readReg(self, address):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.ENC_READ_REG, address, 0, 2)
        except usb.core.USBError:
            print "Could not send ENC_READ_REG vendor request."
        else:
            return ret
    def enc_clearErr(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.ENC_CLEAR_ERR, 0, 0, 2)
        except usb.core.USBError:
            print "Could not send ENC_READ_REG vendor request."
        else:
            return ret

if __name__ == "__main__":
    t = encodertest()
    bias = 97.54
    #print parse_angle(t.enc_setZero())
    hist = [0 for _ in range(100)]
    idx = 0
    full = False
    while True:
        ang = -(parse_angle(t.enc_readReg(t.ENC_ANGLE_AFTER_ZERO_POS_ADDER)) - bias)
        hist[idx] = ang
        idx += 1
        if idx >= 100:
            full = True
            idx = 0

        if full:
            print np.mean(hist)

    
    t.close()

