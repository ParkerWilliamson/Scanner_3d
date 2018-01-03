# Scanner_3d
# A basic 3D scanner made using a line laser and web cam and stepper motor controlled by an Arduino.
# 3d Scanner Main file
# Parker Williamson
# 8/15/16

#To create a 3D scanner using this setup you need:
#  1 line laser
#  1 webcam
#  1 stepper motor
#  1 arduino
#  1 adafruit motor shield
#  1 laptop with all the imported python libraries on it
#  To build a plaform to connect all these different components in a fixed and correctly oriented relation to each other

Python Libs
numpy   #Import matrix lib
cv2           #Import webcam image capture lib
serial        #Import Serial Library
os            #import file naming tool
scanner_class #Import hand written 


Call main.py in the saved dirrectory to run program, some calibration will be required.
