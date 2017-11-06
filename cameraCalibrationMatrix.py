import numpy as np
import cv2
import os                              #import file naming tool
import glob


cam=2
folder = 'C:/Users/Parker/Documents/Python_3d_scanner/Actual_work/collected_Images/MatrixCalcImages'
cam = cv2.VideoCapture(cam)
imgNum=0

##while imgNum<14:
##    # Calulate newest image name
##    imgName = "frame%d.jpg"%imgNum
##    # Capture frame-by-frame
##    ret, frame = cam.read()
##    # Display and save the resulting frame
##    cv2.imshow('frame',frame)
##    if cv2.waitKey(1) == 27: 
##        break  # esc to quit
##    elif cv2.waitKey(1) == 13:
##        #save next image to folder when enter is hit
##        cv2.imwrite(os.path.join(folder, imgName), frame)
##        print "Click: ", imgNum
##        imgNum+=1
##
##cv2.destroyAllWindows()
  
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('C:/Users/Parker/Documents/Python_3d_scanner/Actual_work/collected_Images/MatrixCalcImages/*.jpg')

for fname in images:
    #print(fname)
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6), flags=cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FILTER_QUADS)

##    # Draw and display the corners
##    img = cv2.drawChessboardCorners(img, (7,6), corners,ret)
##    cv2.imshow('img',img)
##    cv2.waitKey(500)
    
    # If found, add object points, image points (after refining them)
    if ret == True:
        print(fname)
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)

#cv2.destroyAllWindows()

# part 2
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

img = cv2.imread(os.path.join(folder, "frame5.jpg"))
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

# undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('calibresult.png',dst)

# error calculator
tot_error = 0
for i in xrange(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    tot_error += error

print "mean error: ", tot_error/len(objpoints)
