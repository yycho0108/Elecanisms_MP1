#!/usr/bin/python

import numpy as np
import cv2
from Queue import Queue
from rectifier import Rectifier
from match import Matcher
from functools import partial

# Low-High Lims for HSV 
handles = ['L_H', 'H_H', 'L_S', 'H_S', 'L_V', 'H_V']
vals = {}

# initial settings
vals['L_H'] = 80
vals['H_H'] = 150
vals['L_S'] = 82
vals['H_S'] = 150
vals['L_V'] = 76
vals['H_V'] = 188

k_erode  = cv2.getStructuringElement(cv2.MORPH_ERODE,(3,3))
k_dilate = cv2.getStructuringElement(cv2.MORPH_DILATE,(3,3))

class CircleHistory:
    def __init__(self, n, thresh_var=10):
        self.n = n
        self.idx = 0
        self.full = False
        self.xq = [None for _ in range(n)]
        self.yq = [None for _ in range(n)] 
        self.rq = [None for _ in range(n)] 

        self.var = thresh_var 

    def put(self,x,y,r):
        if self.idx == self.n:
            self.full = True
            self.idx = 0
        self.xq[self.idx] = x
        self.yq[self.idx] = y
        self.rq[self.idx] = r
        self.idx += 1

    def validate(self):
        if not self.full:
            return False, None
        xv, yv, rv = np.var(self.xq), np.var(self.yq), np.var(self.rq)
        valid_1 = (xv < self.var) and (yv < self.var) and (rv < self.var)

        if valid_1:
            xm, ym, rm = np.mean(self.xq), np.mean(self.yq), np.mean(self.rq)
            x,y,r = self.xq[-1], self.yq[-1], self.rq[-1]
            dx,dy,dr = x-xm, y-ym, r-rm
            valid_2 = (dx*dx < self.var) and (dy*dy < self.var) and (dr*dr < self.var)
            return valid_2, (xm,ym,rm)
        else:
            return False, None 

def process_image(img):
    proc = cv2.GaussianBlur(img,(3,3),0)
    proc = cv2.inRange(img,(vals['L_H'],vals['L_S'],vals['L_V']),(vals['H_H'],vals['H_S'],vals['H_V']))
    proc = cv2.erode(proc,k_erode, iterations=2)
    proc = cv2.dilate(proc,k_dilate, iterations=2)
    return proc



def solve_quad(a,b,c):
    det = b**2-4*a*c
    if det < 0:
        # unsuccessful
        return False, None, None
    else:
        return True, (-b + det)/(2*a), (-b - det)/(2*a)

class Tracker(object):
    def __init__(self):
        self.initialized = False
    def initialize(self, roi, mask,wnd):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        self.roi_hist = cv2.calcHist([hsv],[0],mask,[180],[0,180])
        cv2.normalize(self.roi_hist,self.roi_hist,0,255,cv2.NORM_MINMAX)
        self.term_crit =( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1 ) 
        self.initialized = True
        h,w = mask.shape
        self.wnd = wnd

    def compute(self,frame,draw=False):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv],[0],self.roi_hist,[0,180],1) 
        ret, self.wnd = cv2.CamShift(dst, self.wnd, self.term_crit)
        if draw:
            pts = cv2.boxPoints(ret)
            pts = np.int0(pts)
            box_img = cv2.polylines(frame,[pts],True,255,2)
            print ret[2]
            return ret, box_img
        else:
            return ret, None

def bounds(cx,cy,r):
    minx,maxx= max(cx-r,0), min(cx+r,848)
    miny,maxy= max(cy-r,0), min(cy+r,480)
    return minx,maxx,miny,maxy

class Orienter(object):
    def __init__(self):
        #self.rect = Rectifier('camera.yaml')
        self.history = CircleHistory(10,1000)
        pass
    def get_orientation(self, rho, theta, r, frame=None):
        a = np.cos(theta)
        b = np.sin(theta)
        x0,y0 = a*rho,-b*rho

        if abs(a) > abs(b):
            m = -b/-float(a) # solve with inverse float
            b = x0 - m*y0
            # x = my + b
            
            #verify working:
            #print x0, m*y0+b
            #print x0 + 100*m, m*(y0+100)+b
            cv2.line(frame,(int(x0),-int(y0)),(int(m*(y0+100)+b), -int(y0+100)),(0,255,0),2)
            cv2.line(frame,(int(x0),-int(y0)),(int(m*(y0-100)+b), -int(y0-100)),(0,255,0),2)

            q_a = float(1+m**2)
            q_b = 2*m*(b-r) + 2*r
            q_c = (b-r)**2
            ret, y1,y2 = solve_quad(q_a,q_b,q_c)
            if ret == False:
                return False, -1
            x1,x2 = m*y1+b, m*y2+b
        else: # TODO : FIX
            m = -float(a)/-b # slope, cart
            b = y0-m*x0 # intercept

            #verify working:
            #print y0, m*x0+b
            #print y0 + 100*m, m*(x0+100)+b
            #cv2.line(frame,(int(x0),-int(y0)),(int(x0)+100, -int(m*(x0+100)+_b)),(0,0,255),2)
            #cv2.line(frame,(int(x0),-int(y0)),(int(x0)-100, -int(m*(x0-100)+_b)),(0,0,255),2)

            q_a = float(1+m**2)
            q_b = 2*m*(b+r) - 2*r
            q_c = (b+r)**2

            ret, x1,x2 = solve_quad(q_a,q_b,q_c)
            if ret == False:
                return False, -1
            y1,y2 = m*x1+b, m*x2+b

        if frame != None:
            x1 = int(+ x1)
            y1 = int(- y1)
            x2 = int(+ x2)
            y2 = int(- y2)
            cv2.line(frame,(x1,y1),(x2,y2),(255,0,0),4)
            btm = (x1+x2)/2,(y1+y2)/2
            cv2.line(frame,(r,r),btm,(0,0,255),4)
        return True, np.rad2deg(np.arctan2(-(btm[1]-r),(btm[0]-r)))


    def apply(self, img, draw=False):
        # aniticipating rectified image
        proc = process_image(img)

        cir = cv2.HoughCircles(proc,cv2.HOUGH_GRADIENT, 1, 200, param1=100, param2=30, minRadius=100, maxRadius=500)

        if cir is not None:
            cir = np.uint16(np.around(cir))
            best_cir = cir[0,0] # only the best one

            cx,cy = int(best_cir[0]), int(best_cir[1])
            r = int(best_cir[2]) # radius
            self.history.put(cx,cy,r)
            stable,avg = self.history.validate()

            if stable:
                cx,cy,r = [int(e) for e in avg]
                r_m = int(r*4/3) # r with margin
                r = r_m

                minx,maxx,miny,maxy = bounds(cx,cy,r_m)

                roi = img[miny:maxy, minx:maxx].copy() # region of interest
                roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                roi_proc = proc[miny:maxy, minx:maxx]
                
                mask_ctrs = cv2.findContours(roi_proc.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]
                mask_roi = np.zeros(roi.shape[:-1], dtype=np.uint8)

                for i in range(len(mask_ctrs)):
                    if cv2.contourArea(mask_ctrs[i]) > 20*20:
                        cv2.drawContours(mask_roi,mask_ctrs,i,255,-1)
                roi_masked = cv2.bitwise_and(roi, roi, mask=mask_roi)

                edges = cv2.Canny(mask_roi,0,255,apertureSize=3)

                lines = cv2.HoughLines(edges,1,0.5*np.pi/180,10)

                if lines != None:
                    for rho,theta in lines[0]:
                        res, angle = self.get_orientation(rho,theta,r_m,frame=roi)
                    
                    if res == False:
                        return False, -1, None

                    # Draw Circles After Processing
                    cv2.circle(img, (cx,cy), r,(0,255,0),2)
                    cv2.circle(img, (cx,cy), 2,(0,0,255),3)

                    ctrs = cv2.findContours(roi_proc.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

                    roi_ctr = np.zeros(roi.shape[:-1])
                    for i in range(len(ctrs)):
                        #if cv2.contourArea(ctrs[i]) > 100*100:
                        cv2.drawContours(roi_ctr,ctrs,i,255)
                    return True, angle,(roi,roi_masked,proc,img)
        return False, -1, None 



def on(h, v):
    global vals
    vals[h] = v

if __name__ == "__main__":
    #set flag
    cv2.ocl.setUseOpenCL(False)

    # start camera
    dev = 0 #
    cam = cv2.VideoCapture(dev)

    #initialize window
    win = cv2.namedWindow('Frame');
    ret, img = cam.read()
    cv2.imshow('Frame', img)

    for h in handles:
        # dynamically configure threshold values
        f = partial(on,h)
        cv2.createTrackbar(h,'Frame',vals[h],255,f)

    rect = Rectifier('camera.yaml')
    orienter = Orienter()
    #matcher = Matcher()
    #tracker = Tracker()

    while ret:
        ret,img = cam.read()
        img = rect.apply(img)
        res, angle, frames = orienter.apply(img, draw=True)

        if res:
            print angle
            roi, roi_masked, proc, img = frames
            cv2.imshow('ROI', roi)
            cv2.imshow('ROI_MASKED', roi_masked)
            cv2.imshow('Frame', proc)
            cv2.imshow('Wheel', img)

        k = cv2.waitKey(30)
        if k == 27: # esc
            break

    print 'terminated'
