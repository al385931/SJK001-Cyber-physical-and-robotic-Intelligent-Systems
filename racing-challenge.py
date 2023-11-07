from GUI import GUI
from HAL import HAL
import cv2

# PID Controller Constants
Kp = 0.1  # Proportional constant
Ki = 0  # Integral constant (you can experiment with this)
Kd = 0  # Derivative constant (you can experiment with this)
prev_error = 0.0
integral = 0.0

def calculate_pid(error):
    global prev_error, integral

    proportional = error
    integral += error
    derivative = error - prev_error
    prev_error = error

    return Kp * proportional + Ki * integral + Kd * derivative

i = 0
while True:
    # Enter iterative code!
    img = HAL.getImage()

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv,
                           (0, 125, 125),
                           (30, 255, 255))

    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        M = cv2.moments(contours[0])
        
        if M["m00"] != 0:
            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]
        else:
            cX, cY = 0, 0
        
        error = 320 - cX

        # Calculate control output using PID
        control_output = calculate_pid(error)

        # Set the car's velocity and steering angle
        speed = 2  # Adjust the speed as needed
        steering_angle = 0.01 * control_output  # Adjust the scaling factor as needed

        HAL.setV(speed)
        HAL.setW(steering_angle)

        GUI.showImage(red_mask)
        print('%d cX: %.2f cy: %.2f error: %.2f' % (i, cX, cY, error))
    else:
        # If no contours are found, stop the car
        HAL.setV(0)
        HAL.setW(0)

    i += 1
