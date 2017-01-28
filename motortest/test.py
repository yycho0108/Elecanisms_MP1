import encodertest
import time

enc = encodertest.encodertest()

enc.toggle_led3()

while (True):
    angleBytes = enc.enc_readReg(enc.ENC_ANGLE_AFTER_ZERO_POS_ADDER)

    angle = int(angleBytes[0])+int(angleBytes[1])*256
    print "Bin: {0:016b} Dec:{0:0d}".format(angle)

    time.sleep(.02)