
import mputest
import sys, time

foo = mputest.mputest()
foo.mpu_init()

while 1:
    sys.stdout.write('\x1b[2J\x1b[1;1f')

    accel = foo.mpu_read_accel()
    gyro = foo.mpu_read_gyro()
    mag = foo.mpu_read_mag()

    print 'Accelerometer:'
    print '  x = {0:+05.3f}g'.format(accel[0])
    print '  y = {0:+05.3f}g'.format(accel[1])
    print '  z = {0:+05.3f}g'.format(accel[2])

    print '\nGyroscope:'
    print '  x = {0:+08.3f}dps'.format(gyro[0])
    print '  y = {0:+08.3f}dps'.format(gyro[1])
    print '  z = {0:+08.3f}dps'.format(gyro[2])

    print '\nMagnetometer:'
    print '  x = {0:+08.3f}uT'.format(mag[0])
    print '  y = {0:+08.3f}uT'.format(mag[1])
    print '  z = {0:+08.3f}uT'.format(mag[2])

    t0 = time.clock()
    while time.clock()<t0+0.05:
        pass

