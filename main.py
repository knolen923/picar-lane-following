import cv2
import numpy as np
import time


from picar import front_wheels, back_wheels
from picamera2 import Picamera2
import picar






# Picar setup and getting calibration,
# Important to have the db or car wont move. need django 2.2 or wont work
picar.setup()
DbFile = "/home/pi/SunFounder_PiCar-V/remote_control/remote_control/driver/config"


fw = front_wheels.Front_Wheels(debug=False, db=DbFile)
bw = back_wheels.Back_Wheels(debug=False, db=DbFile)


fw.ready()
bw.ready()
fw.turn_straight()


SPEED = 28








#   STOP COOLDOWN
LastTimeStopp = 0
StopCooldown = 3.0   # seconds to ignore red tape after restarting


#    STEERING
last_angle = 90   # stores previous steering angle




#   red stop tape detection function
def detect_red_tape(image):


   # making image hsv(hue, saturation, value) because easier for red detection
   hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


   # red HSV ranges
   lower_red1 = np.array([0, 70, 70])
   upper_red1 = np.array([10, 255, 255])


   lower_red2 = np.array([170, 70, 70])
   upper_red2 = np.array([180, 255, 255])


   mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
   mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
  
   #combining to build one mask
   red_mask = cv2.bitwise_or(mask1, mask2)


   h, w = red_mask.shape


   # making the roi bottom 60%
   roi = red_mask[int(h * 0.40):h, :]


   #finding contours in the red ROI
   contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)






   # looping through contours and checking if the area is big enough and looks like the red line
   for cnt in contours:


       #filtering by area, removes noise
       area = cv2.contourArea(cnt)
       if area < 500:
           continue




       x, y, width, height = cv2.boundingRect(cnt)


       # checks if its a horizontal strip
       # aspect ratio: width / height > 3, if its not then itll be more pipe shaped so it wont work
       # width > w * .35: basically the red line needs to take up 35% for it to stop, prevents ewarly stoppage
       if width / float(height) > 3.0 and width > w * 0.35:
           return True


   return False






#  yellow lane mask, makes defines the lower and upper range of color(bright yellow)
def yellow_mask(image):


   hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


   lower = np.array([20, 120, 120])
   upper = np.array([40, 255, 255])


   return cv2.inRange(hsv, lower, upper)






# region of interest (ROI) bottom part of the image.
def region_of_interest(image):
   #getting height and width of the image
   #:2 pulls the first two values of the shape function ignore others
   h, w = image.shape[:2]


   #empty mask
   mask = np.zeros_like(image)
   #making polygon that covers the bottom 65 percent of the image,
   # four point polygon
   polygon = np.array([[
       (0, int(h * 0.35)),
       (w, int(h * 0.35)),
       (w, h),
       (0, h)
   ]], dtype=np.int32)


   #filling the pollygon on the mask, pixels inside the polygon are white
   cv2.fillPoly(mask, polygon, 255)


   #applying mask to the image
   return cv2.bitwise_and(image, mask)






#   LANE OFFSET
# takes binary mask, then determines where line is horizontaly,
#  how far it is from center of camera, then how much it should adjust
def find_line_offset(mask, frame_width):


   #computing the image moments,
   # moments: math that describes shape,mass, center of mass of white pixels in image
   moments =cv2.moments(mask)


   #if area of pixels is zero, no line detected, lane lost,
   # returns none so itll fall back to the memory steering previously declared
   if moments["m00"]== 0:
       return None
  
   #computing center of white pixels
   #m10 = sum of all x position white pixels
   #m00= total number of white pixels
   centroid = int(moments["m10"] / moments["m00"])


   #computing the offset from the center of the lane
   return centroid- (frame_width// 2)








#   STEERING CONTROL (SHARPER TURNS)
def steer_with_offset(offset):
   #previously stored steering angle
   global last_angle


   #handling the case of a missing line
   #when no line keep going in the direction it was previously in
   if offset is None:


       fw.turn(last_angle)
       return


   #defining the center steering angle
   center_angle = 90


   # SHARPER TURNS, basically steering sensitivity
   gain = 0.30  


   # computing the steering angle
   # angle < 90 go left
   # angle  > 90 go right
   angle = int(center_angle + offset * gain)


   angle = max(55, min(125, angle))


   fw.turn(angle)


   # Save angle for memory steering
   last_angle = angle




#   DRIVE CONTROL
def drive(offset):
   steer_with_offset(offset)


   if offset is None:
       bw.speed = 22
   else:
       bw.speed =SPEED


   bw.backward()






# camera setup for the ai camera (defining resolutino for the cam as well)
cam = Picamera2()
config = cam.create_preview_configuration(main= {"size": (640, 480)})
cam.configure(config)
cam.start()
time.sleep(0.5)






# main loop for running the car
while True:


   # lots of camera issues so just changing color to be readable and rotate it
   frame = cam.capture_array()
   frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
   frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)




   # red tape detection, making sure a red line wasnt detected within the last 3 seconds before stoppage
   if detect_red_tape(frame) and (time.time() - LastTimeStopp) > StopCooldown:
       print("red tape detected, 3 seconds")


       bw.stop()
       fw.turn_straight()
       time.sleep(3)


       LastTimeStopp = time.time()
       continue


   # making the yellow mask and the roi mask
   mask = yellow_mask(frame)
   cropped = region_of_interest(mask)




   offset = find_line_offset(cropped, frame.shape[1])
   drive(offset)


#end
bw.stop()
fw.turn_straight()
cv2.destroyAllWindows()








