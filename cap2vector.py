#!/usr/bin/env python3

import numpy as np
import cv2 as cv


#Camera parameters - PiCAM 1.0
fx = 2561
fy = 2545.54
cx = 988
cy = 557
pi_mtx = np.array([[ fx,0,cx ],[ 0,fy,cy ],[ 0,0,1 ]],dtype=np.float32)
pi_dist = np.array([ 6.47233420e-02, 1.13627373e+00, 2.73447271e-03, 7.81975172e-03, -5.43129287e+00], dtype=np.float32 )

# Declare variables
window_size = 2.1 #cm
dim_x = 9
dim_y = 7

print("Starting program","\n",".","\n",".","\n")
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
objp = np.zeros((dim_x*dim_y,3), np.float32)
objp[:,:2] = np.mgrid[0:dim_x*window_size:window_size,0:dim_y*window_size:window_size].T.reshape(-1,2)
#print(objp)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    # Color Scheme - BGR
    img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5) #X
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5) #Y
    img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5) #Z
    return img

cap = cv.VideoCapture('/home/torbjoern/Github/prosjektoppgave/ROV_access_point/Video/goodvideo.mp4')
cycle = 1

while(cap.isOpened()):
    print(cycle)
    ret, img = cap.read()
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (dim_x,dim_y),None)
    #cv.imshow("img", img)
    #cv.waitKey(500)
    #cv.destroyAllWindows()
    if ret == True:
        corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        # Find the rotation and translation vectors.
        ret,rvecs, tvecs = cv.solvePnP(objp, corners2, pi_mtx, pi_dist)
        print("RotationVec", rvecs, "TranslationVec", tvecs)
        # project 3D points to image plane
        axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
        imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, pi_mtx, pi_dist)
        img = draw(img,corners2, imgpts)
        #format
        cv.namedWindow('img',cv.WINDOW_NORMAL)
        cv.resizeWindow('img', 800,600)
        cv.imshow('img',img)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        fname = "frame" + str(cycle)
        print(fname)
        cv.imwrite(str(fname+".png"),img)
        cycle += 1
        
    else:
        pass
cap.release()
cv.destroyAllWindows()


