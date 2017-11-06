#Import all the necessary libraries
import matplotlib.pyplot as plt        #import to plot points over an image
import numpy as np                     #Import matrix lib
import cv2                             #Import webcam image capture lib
import serial                          #Import Serial Library
import os                              #import file naming tool
import math                            #needed for sin, cos and sqrt
import time                            #progress bar
import sys                             #progress bar & exit calibration on error

#Holds the necessary methods in order to do all necessary functions of a scanner
class scanner:
   '''A class representing a 3d scanner's image collecting, processing and shape model'''
   def __init__(self):
      print('Hello, Mighty One')
 
 
   #onePointCalibrator finds the axis of the scanner setup, by finding the average horizontal pixel of the pixel line halfway up the picture
   def onePointCalibrator(self, cam, file):
      #initiate webcam
      video = cv2.VideoCapture(cam)
      #save an image from the webcam
      ret, singleCapture = video.read()
      #shows the photo on screen
      cv2.imshow('calibration',singleCapture)
      cv2.waitKey(100)
      #save and opens the image in the selected file under the name saved in imgTitle (needed to use the shape command)
      imgTitle = "Single_Calibration_Image.jpg"
      cv2.imwrite(os.path.join(file, imgTitle), singleCapture)
      singleImg = cv2.imread(os.path.join(file, imgTitle))
      #find the vertical center pixel line of the photo
      [y, x]=np.shape(singleImg[:,:,0])
      midHeight=int((y-1)/2)
      #showImage selects weather the processed image is shown, since it will pause the program
      showImage= True
      print('exit filtered image to continue')
      #calls image_Processing which calculates and returns an array of the horizontal center points of the rows that 
      #include any red and an array of the assosciated row numbers that include red
      [mid, rowWithPoints]=self.image_Processing(singleCapture, 1, showImage)
      rowWithPoints = np.array(rowWithPoints)
      
      #find the array location that correlates to the mid height of the photo
      location = np.where(rowWithPoints==midHeight)
      #Find the horizontal mid point of the row that is half way up the photo
      centerLine= mid[location]
      video.release()
      cv2.destroyAllWindows()
      #If 'IndexError: out of bounds' here most likely did not find a red point half way up image representing the axis of rotation
      
      #find the platform center point finds where the newest point starts to move left faster(since that is where the vertical calibration line hits the platform)
      axis = []
      for point in range(0,len(mid)):
         axis = np.append(axis, mid[point])
         if point != 0 and point != 1:
            avgOfLastThreePoints = (axis[point]+axis[point-1]+axis[point-2])/3
            if mid[point] <= avgOfLastThreePoints - 2:
               baseAxis = mid[point-3]
               platformLevel = rowWithPoints[point-3]
               break
         
      centerline = np.sum(axis)/len(axis)   
      #print("center: {}".format(centerline))
   
      #shows the image
      implot = plt.imshow(singleImg)

      # put a red dot at the midpoint of each row
      plt.scatter(x=baseAxis, y=platformLevel, c='g')

      plt.plot([centerline,centerline],[0,platformLevel],'-', c='g')
      plt.show()

      return [centerLine, platformLevel]
         

   
      
   def collect_Image(self, cam, arduinoSerialData, numberOfPhotos, file):
      cam = cv2.VideoCapture(cam)     #initiates video capture of selected camera
      captureImgArbString = "Images captured"
      for num in range(1,numberOfPhotos+1):
         #Calls the method that shows a progress bar of how many of the photos have been taken (how far through the range of photos the loop is)
         self.update_progress((num/float(numberOfPhotos)),captureImgArbString)
         movementFlag = 0 
      
         # Capture frame-by-frame
         ret, frame = cam.read()
         #calulate newest image name
         imgName = "frame%d.jpg"%num      
         # Display and save the resulting frame
         cv2.imshow('frame',frame)
         if cv2.waitKey(1) == 27: 
			   break  # esc to quit
         cv2.imwrite(os.path.join(file, imgName), frame)        
         arduinoSerialData.write(b'1')
         
         #print("waiting")
         #wait for a response of the stepper's motion from the arduino
         while(movementFlag==0):
            movementFlag = int(arduinoSerialData.read())
            
         #print("received")   
      # When everything done, release the capture
      cam.release()
      cv2.destroyAllWindows()
      return;
   
   
   def plotting_3d(self, numberOfPix, camAng, file, axisOfRotation, camHeight, lengthFromCam, v_0):
      rotationPerImage = 2*math.pi/numberOfPix        #rad/image 
      x = []
      y = []
      z = []
      pixelToInchScale = 3.0625/190.05              #calibrate per setup using a chess grid in x and y dirrections (make sure theyre the same)
      pixArbString = "Images analyzed"
      for imgNum in range(0,numberOfPix):
         #Calls the method that shows a progress bar of how many of the photos have been analyzed (how far through the range of photos the loop is)
         #self.update_progress((imgNum/float(numberOfPix-1)),pixArbString)
         
         #calulate image name and open it
         imgName = "frame%d.jpg"%(imgNum+1)
         img = cv2.imread(os.path.join(file, imgName))
         #print(os.path.join(file, imgName))
         #cv2.imshow(imgName, img)
         [imgHeight, imgWidth] = np.shape(img[:,:,0])
         
         #declare variable for image processing (may want to move to main eventually)
         showImage= False
         verticalResolution = 1
         
         #calls image_Processing which calculates and returns an array of the horizontal center points of the rows that 
         #include any red and an array of the assosciated row numbers that include red
         [AvgMid, yPos] = self.image_Processing( img, verticalResolution, showImage)
         d = AvgMid - axisOfRotation #make locations based on distance from the axis of rotation rather than side of frame
         v = imgHeight - yPos #make locations poitive from bottom of frame
         
#          print("dlen = {}".format(len(d)))
#          print("vlen = {}".format(len(v)))
         
         #scale measurements from pixels to inches
         d = d*pixelToInchScale
         v = v*pixelToInchScale
         
         #calculates the absolute x, y and z cordinates from the x and y points measured from the bottom of the image and axis of rotation
         #X and Y
         with np.errstate(divide='ignore', invalid='ignore'):
            c = np.true_divide(lengthFromCam,d)
            c[c == np.inf] = 0
            c = np.nan_to_num(c)
         beta = np.arctan(c)
         rho = 120 - beta
         ray = d*(np.sin(beta)/np.sin(rho))
         d = ray*np.sin(camAng)
   
         a = ray*np.sin(camAng)                                                                   #np.multiply(d, math.cos(camAng))
         b = ray*np.cos(camAng)                                     #np.multiply(d, math.sin(camAng))
         dist = ray
         x= np.append(x, (dist*np.cos((imgNum)*rotationPerImage)), axis=0)
         y= np.append(y, (dist*np.sin((imgNum)*rotationPerImage)), axis=0) 
         
         #Z
         length=lengthFromCam-b        #b=not shifted y
         theta1=np.arctan(length/camHeight)
         thetaTotal=np.arctan(lengthFromCam/camHeight)
         theta2=thetaTotal-theta1
         
         deltaV = v - v_0
         TY = deltaV*((np.sin((np.pi/2)-theta2))/(np.sin((np.pi/2)-theta1)))
         deltaY = TY - b               #b=not shifted y
         deltah = deltaY*(np.tan((np.pi/2)-theta1))
         
         z= np.append(z, deltah, axis=0)       #append vertically
      return [x,y,z]



   def image_Processing(self, img, verticalResolution, showImage):
      #filter out all but the red color of the images
      redImg = img[:,:,2]
      threshold=200

      # Otsu's thresholding after Gaussian filtering
      blur = cv2.GaussianBlur(redImg,(5,5),0)
      ret3,filteredImg = cv2.threshold(blur,threshold,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
      
      #find locations where the binary image is not zero
      [rowLoc, colLoc] = np.nonzero(filteredImg)
      
      #gets the image size
      [ySize,xSize] = np.shape(filteredImg)
      
      #variable for image processing loop to store the results
      AvgMid = []
      rowsWithPoints = []
      total=np.zeros((ySize,1))
      pointsPerRow=np.zeros((ySize,1))
      row = 0;
       
      for i in range(0,len(rowLoc)-1):
         #finds the data of the pixel being processed
         row = rowLoc[i]
         total[row,0] = total[row,0] + colLoc[i]
         pointsPerRow[row,0] = pointsPerRow[row,0]+1
         
         #if next point in in the next row (ie. is the last point of a row AND is a verticalResolutionTH row (to speed up process can shrink for higher resolution)
         if (rowLoc[i+1])>row and (row%verticalResolution==0):
            #if the avgMid point exists continue
            if np.size(AvgMid)==0:
               rowsWithPoints=[row]
               avg = total[row,0]/float(pointsPerRow[row,0])
               AvgMid=[avg]
            else:   
               rowsWithPoints = np.concatenate((rowsWithPoints, np.array([row])), axis=0)
               avg = total[row,0]/float(pointsPerRow[row,0])
               AvgMid = np.concatenate((AvgMid, np.array([avg])), axis=0)
      
      #if flag says to show the image
      if (showImage==True):
         #shows the image
         implot = plt.imshow(filteredImg)

         # put a red dot at the midpoint of each row
         plt.scatter(x=AvgMid, y=rowsWithPoints, c='g') 
         plt.show()
      
      return [AvgMid, rowsWithPoints]
      
      
   #I found this online will produce a progress bar of the selected variable   
   def update_progress(self, progress, arbString):
      barLength = 20 # Modify this to change the length of the progress bar
      status = ""
      if isinstance(progress, int):
         progress = float(progress)
      if not isinstance(progress, float):
         progress = 0
         status = "error: progress var must be float\r\n"
      if progress < 0:
         progress = 0
         status = "Halt...\r\n"
      if progress >= 1:
         progress = 1
         status = "Done...\r\n"
      block = int(round(barLength*progress))
      text = "\r{0}: [{1}] {2}% {3}".format(arbString, "#"*block + "-"*(barLength-block), progress*100, status)
      sys.stdout.write(text)
      sys.stdout.flush()
      return
      
   #shows what the camera is seeing to allow for user to position as desired   
   def viewCamera(self, cam, file):
      cam = cv2.VideoCapture(cam)     #initiates video capture of selected camera
      print("press esc once camera is centered")
      imgName = "Positioning_Picture.jpg"
      while (True==True):
         ret_val, vid = cam.read() # Display and save the resulting frame
         cv2.imshow('frame',vid)
         cv2.imwrite(os.path.join(file, imgName), vid)
         if cv2.waitKey(1) == 27: 
            break  # esc to quit
      cv2.destroyAllWindows()      
      return
      
