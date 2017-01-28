import encodertest
import time

enc = encodertest.encodertest()
enc.toggle_led3()
#print enc.enc_clearErr()

while (True):
    #print enc.enc_readReg(enc.ENC_MAGNITUDE)
    #print enc.enc_readReg(enc.ENC_ANGLE_AFTER_ZERO_POS_ADDER)
    angleBytes = enc.enc_readReg(enc.ENC_ANGLE_AFTER_ZERO_POS_ADDER)
    cycle = 0x1FFF
    angle = int(angleBytes[0])+int(angleBytes[1])*256
    print "Bin: {0:016b} Hex:{1:04x} Dec:{2:0f} Angle:{3:0f}".format(angle, angle, float(angle), float(angle)/cycle*360)
    time.sleep(.02)
