from GUI import GUI
from HAL import HAL
import cv2
# Enter sequential code!

# PID Controller Constants

prev_error_angular = 0.0
prev_error_braking = 0.0

integral_angular = 0.0
integral_braking = 0.0


brake = 0

curve_angle = 0

def calculate_pid_angular(error):
    global prev_error_angular, integral_angular
    Kp = 0.2  # Proportional constant
    Ki = 0.0001 # Integral constant (you can experiment with this)
    Kd = -0.00002  # Derivative constant (you can experiment with this)
    proportional = error
    integral_angular += error
    derivative = error - prev_error_angular
    prev_error = error

    return Kp * proportional + Ki * integral_angular + Kd * derivative

def calculate_pid_braking(error):
    global prev_error_braking, integral_braking
    Kp = 0.5  # Proportional constant
    Ki = 0  # Integral constant (you can experiment with this)
    Kd = 0  # Derivative constant (you can experiment with this)
    proportional = error
    integral_braking += error
    derivative = error - prev_error_braking
    prev_error = error

    return Kp * proportional + Ki * integral_braking + Kd * derivative


def calculate_speed_factor(curve_angle):
    return max(0.5, 1 - 0.1 * abs(curve_angle))
    
i = 0
cX, cY = 0,0
        
while True:
    # Enter iterative code!
    img = HAL.getImage()
    

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv,
                      (0, 125, 125),
                      (30, 255, 255))
                         
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)

    # Encontrar el centroide de la figura formada por los contornos
        M = cv2.moments(contours[0])
        if M["m00"] != 0:
            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]
        else:
            cX, cY = 0, 0

        # Calcular el error desde el centroide hasta el centro de la pantalla
        error = 320 - cX
        if i == 0:
            speed = 7 
            steering_angle = 0# Adjust the speed as needed
    
        if (cX >0):# Calcula el ángulo de la curva
            ellipse = cv2.fitEllipse(max_contour)
            curve_angle = ellipse[2]

      # Calculate control output using PID
            
         # Set the car's velocity and steering angle
            if i > 0:
                control_output_angular = calculate_pid_angular(error)
                steering_angle = 0.01*control_output_angular 
                if  abs(error) > 230:
                    error_brake = abs(error)
                    brake = 0.002*calculate_pid_braking(error_brake)
                    speed = min(max(0.8,speed-brake),2.5)
                if abs(curve_angle) > 10 and abs(error) < 230:
                    error_brake = abs(error)
                    brake = 0.002*calculate_pid_braking(error_brake)
                    speed = min(max(2.5,speed+brake),3.7)
                    #speed = max(1, 5 * calculate_speed_factor(curve_angle) * (1 - 0.2 * abs(steering_angle)))
                if abs(curve_angle)<= 10 and abs(steering_angle)<0.05:
                    #speed = max(3.5, 7 * calculate_speed_factor(curve_angle) * (1 - 0.2 * abs(steering_angle)))
                    error_brake = abs(error)
                    brake = 0.01*calculate_pid_braking(error_brake)
                    speed = min(max(3.7,speed+brake),7)
            HAL.setW(steering_angle) 
            HAL.setV(speed)
             
        i = i+1  
        #GUI.showImage(red_mask)
        #print('%d angle: %.2f cX: %.2f cy: %.2f speed: %.2f error: %.2f curve: %.2f  ' % ( i,steering_angle, cX, cY, speed, error, curve_angle))
        print('%d angle: %.2f speed: %.2f error: %.2f curve: %.2f brake: %.2f ' % ( i,steering_angle, speed, error, curve_angle,brake))
    else:
        # If no contours are found, stop the car
        HAL.setV(0)
        HAL.setW(0)     
        
        
    GUI.showImage(red_mask)
