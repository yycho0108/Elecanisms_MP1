import numpy as np
import cv2

MIN_MATCH_COUNT = 10

from cmath import rect,phase
from math import radians, degrees

def mean_angle(deg):
    return degrees(phase(sum(rect(1, radians(d)) for d in deg)/len(deg))) 

def drawMatches(img1, kp1, img2, kp2, matches):
    """
    My own implementation of cv2.drawMatches as OpenCV 2.4.9
    does not have this function available but it's supported in
    OpenCV 3.0.0

    This function takes in two images with their associated 
    keypoints, as well as a list of DMatch data structure (matches) 
    that contains which keypoints matched in which images.

    An image will be produced where a montage is shown with
    the first image followed by the second image beside it.

    Keypoints are delineated with circles, while lines are connected
    between matching keypoints.

    img1,img2 - Grayscale images
    kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint 
              detection algorithms
    matches - A list of matches of corresponding keypoints through any
              OpenCV keypoint matching algorithm
    """

    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1] = img1 #np.dstack([img1, img1, img1])

    # Place the next image to the right of it
    out[:rows2,cols1:] = img2 #np.dstack([img2, img2, img2])

    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for mat in matches:

        # Get the matching keypoints for each of the images
        img1_idx = mat.queryIdx
        img2_idx = mat.trainIdx

        # x - columns
        # y - rows
        (x1,y1) = kp1[img1_idx].pt
        (x2,y2) = kp2[img2_idx].pt

        # Draw a small circle at both co-ordinates
        # radius 4
        # colour blue
        # thickness = 1
        cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
        cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

        # Draw a line in between the two points
        # thickness = 1
        # colour blue
        cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0), 1)

    # Also return the image if you'd like a copy
    return out

class Matcher(object):
    def __init__(self,ref_frame=None):
        # Descriptor
        #orb = cv2.ORB_create(1000,patchSize=31)
        orb = cv2.ORB_create()

        #brisk = cv2.BRISK_create()

        # Matcher
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        #bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # choose descriptor and matcher
        self.descriptor = orb 
        self.matcher = flann
        # Initialize Keypoints
        self.has_ref = False
        if ref_frame:
            self.set_ref(ref_frame)

    def set_ref(self,ref_frame):
        self.has_ref = True
        self.ref_frame = ref_frame.copy()
        self.kp_ref,self.des_ref = self.descriptor.detectAndCompute(self.ref_frame, None)
        self.h,self.w = ref_frame.shape[:-1]
        #uncomment if match_2
        #self.ref_rots = [cv2.warpAffine(self.ref_frame, cv2.getRotationMatrix2D((self.w/2,self.h/2),deg,1),(self.w,self.h))
        #        for deg in range(0,360)]
    def match_3(self,img):
        kp1, des1 = self.kp_ref, self.des_ref
        kp2, des2 = self.descriptor.detectAndCompute(img,None)
        if(des1.dtype != np.float32):
            des1 = des1.astype(np.float32)
        if(des2.dtype != np.float32):
            des2 = des2.astype(np.float32)
        matches = self.matcher.knnMatch(des1,des2,k=2)
        # store all the good matches as per Lowe's ratio test.
        good = []
        obj = []
        scene = []

        for m,n in matches:
            if m.distance < 0.8*n.distance:
                good.append(m)
                obj.append(kp1[m.queryIdx].pt)
                scene.append(kp2[m.trainIdx].pt)

        #tf = cv2.estimateRigidTransform(np.array(scene),np.array(obj),False)
        tf = cv2.estimateRigidTransform(img, self.ref_frame, False)

        if tf != None:
            print tf
            match_img = cv2.warpAffine(self.ref_frame,tf,dsize=img.shape[:-1],borderMode=cv2.BORDER_DEFAULT)
            return True, match_img
        else:
            return False, None

    def match_2(self,img,res=1):
        method = cv2.TM_CCOEFF_NORMED
        idx = 0
        cur_max = 0.
        for i, ref in enumerate(self.ref_rots):
            if (i % res) != 0:
                continue
            ret = cv2.matchTemplate(img,ref,method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(ret)
            if max_val > cur_max:
                cur_max = max_val
                idx = i
        return idx

    def match(self,img,draw=False):
        # return orientation
        kp1, des1 = self.kp_ref, self.des_ref
        kp2, des2 = self.descriptor.detectAndCompute(img,None)
        try:
            if(des1.dtype != np.float32):
                des1 = des1.astype(np.float32)
            if(des2.dtype != np.float32):
                des2 = des2.astype(np.float32)
            matches = self.matcher.knnMatch(des1,des2,k=2)
            # store all the good matches as per Lowe's ratio test.
            good = []
            obj = []
            scene = []

            for m,n in matches:
                if m.distance < 0.8*n.distance:
                    good.append(m)
                    obj.append(kp1[m.queryIdx].pt)
                    scene.append(kp2[m.trainIdx].pt)

            H,mask = cv2.findHomography(np.array(obj),np.array(scene),cv2.RANSAC)
            h,w = self.ref_frame.shape[:-1]
            obj_corners = np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
            scene_corners = cv2.perspectiveTransform(obj_corners, H)

            if draw:
                #match_img = drawMatches(self.ref_frame,kp1,img,kp2,good)
                match_img = img.copy()
                cv2.polylines(match_img, [np.int32(scene_corners)], True,255,5,cv2.LINE_AA)
                return True, match_img
            else:
                return False, None

        except Exception as e:
            print 'e : ', e
            return False, None

if __name__ == "__main__":
    matcher = Matcher()

    freeze = False

    imgs = []
    img1 = None

    cam = cv2.VideoCapture(0)

    while True:
        _, img = cam.read()
        imgs.append(img)
        if(len(imgs) > 10):
            del imgs[0]

            if not freeze:
                img1 = imgs[0][320:,240:]

            same,match_frame = matcher.match(img1, imgs[-1],draw=True)

            if same:
                cv2.imshow('matches',match_frame)
            else:
                print 'no matches'

        k = cv2.waitKey(20)
        if k == 27:
            break
        elif k == 32:
            freeze = True
            matcher.set_ref(img1)
