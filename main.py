#3d Scanner Main file
#Parker Williamson
#8/15/16

#To create a 3D scanner using this setup you need:
#  1 line laser
#  1 webcam
#  1 stepper motor
#  1 arduino
#  1 adafruit motor shield
#  1 laptop with all the imported python libraries on it
#  To build a plaform to connect all these different components in a fixed and correctly oriented relation to each other

import numpy as np   #Import matrix lib
import cv2           #Import webcam image capture lib
import serial        #Import Serial Library
import os            #import file naming tool
import scanner_class #Import hand written 

#Creates a scanner object which can be called using methods in order to take in data
a = scanner_class.scanner()

#Creates variables used to control the single scanner this file uses (the variables may need to change with different setups)
cam=2
folder = 'C:\Users\Parker\Documents\Python_3d_scanner\Actual_work\collected_Images'
numberOfPhotos = 400 #number of steps of the stepper motor per revolution
camAng = 35  #deg
arduinoSerialData = serial.Serial('COM3', 9600)
camHeight = 2.875          #inches
lengthFromCam = 25          #inches
axisOfRotation = 292.5 #calibration needed (after calibration is completed put calibration value here and skip calibration step)
v_0 = 371

####Quiries the user about whether they want to calibrate the scanners setup (will be needed if it has not been done with the current setup).
####In order to skip this Quiry you can comment out from HERE (to next instance of 'HERE')
##import time
##from threading import Thread
##entry = None
##def check():
##    time.sleep(5)
##    if entry=='y' or entry=='Y' or entry=='Yes' or entry=='yes' or entry=='YES':
##       print("Calibrating")
##       #Show the output of the camera in order for the user to center image as desired (comment out after setup and immobilization
##       a.viewCamera(cam, folder)
##       #Calls a method that finds the axis of the scanner setup, by finding the average horizontal pixel of the points above the platform
##       [axisOfRotation, v_0] = a.onePointCalibrator(cam, folder)
##       print('Axis of Rotation (by horizontal pixel number): {}\n'.format(axisOfRotation))
##       print('v_0 (by horizontal pixel number): {}\n'.format(v_0))
##       print('Ammend code  in main.py with the calibrated axisOfRotation and v_0 in order to skip \nthis step in the future')
##       throwAway = raw_input('hit enter to continue once the object to be scanned has been \nplaced on the scanner')
##       return
##    print ("\nCalibration skipped")
##    
##        
##Thread(target = check).start()
##entry = raw_input('Calibrate prior to data collection (y/n)(10 sec timer, if missed hit "ctrl-C"): ')
###End skipping calibration comment HERE

#exit()
##import csv
###save calibration data to a folder and just ask if calibration is needed
##with open('calibrationFile.csv') as csvfile:
##     reader = csv.DictReader(csvfile)     
##     Axis = row['Axis']
##import json
##json.loads(calibration.json)


#Quiries the user about whether they want to calibrate the scanners setup (will be needed if it has not been done with the current setup).
#In order to skip this Quiry you can comment out from HERE (to next instance of 'HERE')
entry = raw_input('Calibrate prior to data collection (y/n): ')

#If any of the below is not entered, the program assumes calibration is not needed
if entry=='y' or entry=='Y' or entry=='Yes' or entry=='yes' or entry=='YES':
    #Show the output of the camera in order for the user to center image as desired (comment out after setup and immobilization
    a.viewCamera(cam, folder)
 
    
    #Calls a method that finds the axis of the scanner setup, by finding the average horizontal pixel of the points above the platform
    [axisOfRotation, v_0] = a.onePointCalibrator(cam, folder)
    print('Axis of Rotation (by horizontal pixel number): {}\n'.format(axisOfRotation))
    print('v_0 (by horizontal pixel number): {}\n'.format(v_0))
    print('Ammend code  in main.py with the calibrated axisOfRotation and v_0 in order to skip \nthis step in the future')
    throwAway = raw_input('hit enter to continue once the object to be scanned has been \nplaced on the scanner')
    #exit()           #use when working of fixing calibration
 
 #End skipping calibration comment HERE

#Calls a method from scanner class in order to collect a full stepper motor revolution of photos and saves them to a selected folder/file
a.collect_Image(cam, arduinoSerialData, numberOfPhotos, folder)

#Calculates the x, y and z points of the selected pictures
[x,y,z] = a.plotting_3d(numberOfPhotos, camAng, folder, axisOfRotation, camHeight, lengthFromCam, v_0);    #calls function to stitch together all the data from the selected file/photos

#Saves the x, y and z points in an easily accesible .txt formate
fileName = "output.txt"
mainFolder = 'C:\Users\Parker\Documents\Python_3d_scanner\Actual_work'
coordinates = open(os.path.join(mainFolder, fileName), 'w')
pointArbString = "Points saved to text file"
for point in range(0,len(x)):
   a.update_progress((point/float(len(x)-1)),pointArbString)
   coordinates.write('{} {} {}\n'.format(x[point], y[point], z[point]))
print("\nDONE???")
coordinates.close()
