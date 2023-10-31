from GUI import GUI
from HAL import HAL
import cv2
# Enter sequential code!

i=0
while True:
    # Enter iterative code!
    img = HAL.getImage()
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv,
                      (0, 125, 125),
                      (30, 255, 255))
    

    GUI.showImage(red_mask)
    print(i)
    i=i+1
    
