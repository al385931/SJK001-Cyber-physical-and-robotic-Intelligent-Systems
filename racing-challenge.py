from GUI import GUI
from HAL import HAL
import cv2
# Enter sequential code!

i=0
# PID Controller Constants

prev_error_angular = 0.0
prev_error_braking = 0.0
prev_error_accel = 0.0

integral_angular = 0.0
integral_braking = 0.0
integral_accel = 0.0

initial_speed = 3.9

def calculate_pid_angular(error):
    global prev_error_angular, integral_angular
    Kp = 0.3  # Proportional constant
    Ki = 0  # Integral constant (you can experiment with this)
    Kd = 0  # Derivative constant (you can experiment with this)
    proportional = error
    integral_angular += error
    derivative = error - prev_error_angular
    prev_error = error

    return Kp * proportional + Ki * integral_angular + Kd * derivative

def calculate_pid_braking(error):
    global prev_error_braking, integral_braking
    Kp = 1  # Proportional constant
    Ki = 0.1  # Integral constant (you can experiment with this)
    Kd = 0.1  # Derivative constant (you can experiment with this)
    proportional = error
    integral_braking += error
    derivative = error - prev_error_braking
    prev_error = error

    return Kp * proportional + Ki * integral_braking + Kd * derivative

def calculate_pid_accel(error):
    global prev_error_accel, integral_accel
    Kp = 0.2  # Proportional constant
    Ki = 0  # Integral constant (you can experiment with this)
    Kd = 0  # Derivative constant (you can experiment with this)
    proportional = error
    integral_accel += error
    derivative = error - prev_error_accel
    prev_error = error

    return Kp * proportional + Ki * integral_accel + Kd * derivative
    
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
        control_output_angular = calculate_pid_angular(error)
        # Set the car's velocity and steering angle
        if i == 0:
          speed = initial_speed+calculate_pid_accel(0.1)  # Adjust the speed as needed
          steering_angle = 0.01 * control_output_angular# Adjust the scaling factor as needed
        else:
          if speed >= 4:
              error_brake = abs(speed-2)
              speed = speed-calculate_pid_braking(error_brake)
          else:
              error_accel = abs(speed-2)
              speed = speed+calculate_pid_accel(error_accel)
              
        steering_angle = 0.01 * control_output_angular 
        HAL.setV(speed)
        HAL.setW(steering_angle)
        GUI.showImage(red_mask)
        print('%d cX: %.2f cy: %.2f' % (i,cX, cY))
        
    else:
        # If no contours are found, stop the car
        HAL.setV(0)
        HAL.setW(0)
    i = i+1
