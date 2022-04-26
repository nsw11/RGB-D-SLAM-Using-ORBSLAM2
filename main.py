import cv2
import numpy as np
from openni import openni2
from openni import _openni2 as c_api
import os
import time

openni2.initialize()
dev = openni2.Device.open_any()

# Start the depth stream
depth_stream = dev.create_depth_stream()
depth_stream.start()
depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM, resolutionX = 640, resolutionY = 480, fps = 30))

# Start the colour stream
#color_stream = dev.create_color_stream()
#color_stream.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, resolutionX = 640, resolutionY = 480, fps = 30))
#color_stream.start()
#openni2.wait_for_any_stream([color_stream])


cap = cv2.VideoCapture(2)  #def = 2, change if camera port is changes
if not cap.isOpened():
    print("Cannot open camera")
    exit()


test_name = 'test3' # generates a directory to place all files in for this test

f1 = open(test_name+"/depth.txt","w")
f2 = open(test_name+"/rgb.txt","w")

while True:
                # Grab frame
                ret, frame_ = cap.read()   #rgb frame
                frame_ = cv2.resize(frame_,(640,480), interpolation = cv2.INTER_AREA) #resize to 640x480
                ts = time.time()  #get current time for rgb

                frame = depth_stream.read_frame()  #depth frame
                frame_data = frame.get_buffer_as_uint16()
                img = np.frombuffer(frame_data, dtype=np.uint16)
                ts_dp = time.time()                #get current time for depth
                img.shape = (1, 480, 640)
                img = np.concatenate((img, img, img), axis=0)
                img = np.swapaxes(img, 0, 2)
                img = np.swapaxes(img, 0, 1)
                
                
                #generate file name using the timestamps
                #*************RGB*******************
                rgb_filename = str(ts)+".png"
                Image_rgbpath = test_name+"/colour/" + str(rgb_filename)
                rgbpath = "colour/" + str(rgb_filename)
                print(rgbpath)

                #*************depth*******************
                depth_filename = str(ts_dp)+".png"
                Image_dpath = test_name+"/depth/" + str(depth_filename)
                dpath = "depth/" + str(depth_filename)
                print(dpath)

                #display image on the screen
                cv2.imshow('RGB image', frame_)  #display rgb image
                cv2.imshow("Depth Image", img)   #display depth image
                key = cv2.waitKey(1) & 0xFF

                #store the images
                cv2.imwrite(str(Image_rgbpath),frame_)  #rgb
                cv2.imwrite(str(Image_dpath),img)       #depth

                #write to depth and rgb files
                f1.write(str(ts_dp) +" "+ dpath + "\n")
                f2.write(str(ts) +" "+ rgbpath +"\n")


# Close all windows and unload the depth device
openni2.unload()
cv2.destroyAllWindows()
f1.close()
f2.close()
