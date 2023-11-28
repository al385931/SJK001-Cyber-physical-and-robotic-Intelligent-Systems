from GUI import GUI
from HAL import HAL
import cv2
import math
import numpy as np

## Queda bastante por hacer, puede que la espiral esté mal hecha.
## Hay que hacer que el dron se mueva un poco más decente. Ya que al no estar estabilizado le puede costar mucho detectar una imagen.
## Falta almacenar la posición gps de la cara. 
## Faltaría también volver a tierra.

# Enter sequential code!
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
VICTIMS_X = 30.35
VICTIMS_Y = -40.3
ship_utm = (492,162)
survivors_utm = (532, 132)
ship_world = (0,0)
survivors_world = (survivors_utm[0]-ship_utm[0], survivors_utm[1]-ship_utm[1])

x_vel = 0.25
angle = 0.6
iterations = 0
spiral_iterations = 300
landin_margin = 0.07

#search area definition 
spiral_separation = 4 #we want that the radius expand 4 metres in each rotation
angle_step = math.pi / 4 # 45 degrees per step for a rapid spiral expansion
num_steps = 45
fly_height = 5 
search_path = []

#Here we will save the coordinates of the survivors.
survivor_cords=[]

#Returns the actual position of the drone as a numpy array [x, y, z], in m.
x_pos = HAL.get_position()[0]
y_pos = HAL.get_position()[1]

#Takeoff at the current location, to the given height
HAL.takeoff(5)

#First here is searching the people
while not ((VICTIMS_X -1 < x_pos) and (x_pos < VICTIMS_X +1 ) and (VICTIMS_Y -1 < y_pos) and (y_pos < VICTIMS_Y +1 )):
    GUI.showImage(HAL.get_frontal_image())
    GUI.showLeftImage(HAL.get_ventral_image())
    x_pos = HAL.get_position()[0]
    y_pos = HAL.get_position()[1]
    HAL.set_cmd_pos(VICTIMS_X, VICTIMS_Y, 5, angle)
    #Commands the position (x,y,z) of the drone, in m and the yaw angle (az) (in rad) taking as reference the first takeoff point (map frame)

	
      
  #time.sleep(0.01)
  
# Generate the search path. Tengo dudas de si esto se utilizará para el comando de mover o para simplemente decir donde tiene que ir.
for step in range(num_steps):
    radius = step * spiral_separation
    angle = step * angle_step
    x = radius * math.cos(angle)+ survivors_world[0] #coordenates are related to the initial position
    y = radius * math.sin(angle) + survivors_world[1]
    z = fly_height
    search_path.append((x, y, z))

#now we are going to tell the dron, how to move. #first we we'll define a function that indicates if we are in the desired 
#position
def drone_in_pos(current_goal, int = 3.0):
    x_pos = HAL.get_position()[0]
    y_pos = HAL.get_position()[1]		
    if ((current_goal[0] -1 < x_pos) and (x_pos < current_goal[0] +1 ) and (current_goal[1]-1 < y_pos) and (y_pos < current_goal[1] + 1 ) and (abs(HAL.get_yaw())< int)):
        return True
    else:
        return False
        
def rotate_image(img, angle):
    height, width = img.shape[:2]
    # Calculate the rotation matrix
    rotation_matrix = cv2.getRotationMatrix2D((width/2, height/2), angle, 1)
    # Apply the rotation to the image
    rotated_image = cv2.warpAffine(img, rotation_matrix, (width, height))
    return rotated_image
    
def location_survivor(face_cords):
    global survivor_cords
    #Here, we check if is a repeated face (we check if we have a very close coords)
    if len(survivor_cords) > 1 :
        for cordenates in survivor_cords:
            if abs(face_cords[0]-cordenates[0])<2 and (face_cords[1] - cordenates[1])<2:
                survivor_cords.append((face_cords[0], face_cords[1]))
                return print(f"There is a survivor in ({face_cords[0]}, {face_cords[1]})")
            else:
                return print(f"We've already know the coordinates of this survivor")
    else:
        survivor_cords.append((face_cords[0], face_cords[1]))
        return print(f"There is a survivor in ({face_cords[0]}, {face_cords[1]})")

def check_survivors():
    faces = []
    img = HAL.get_ventral_image()
    for angle in range(0, 360, 10):
        rotated_img = rotate_image(img, angle)
        gray = cv2.cvtColor(rotated_img, cv2.COLOR_BGR2GRAY)
        faces.append(face_cascade.detectMultiScale(gray, 1.1, 4))
#aqui habría que almacenar la posición del superviviente.
    if len(faces)>=1:
        x_cord = HAL.get_position()[0]
        y_cord = HAL.get_position()[1]
        return location_survivor((x_cord, y_cord))
    else:
        return None

#este while tendrá dos condiciones, uno centrado en la posicion en la que està y otro en el número the supervivientes encontrado.
pos_commanded = False
while True:
    check_survivors()
    current_goal = search_path[0]
    print(current_goal)
    if not pos_commanded: 
        HAL.set_cmd_pos(current_goal[0], current_goal[1], fly_height, 0)
        pos_commanded = True
    elif drone_in_pos(current_goal, 3.0):
       search_path = search_path[1:]
       pos_commanded = False
    GUI.showImage(HAL.get_frontal_image())
    GUI.showLeftImage(HAL.get_ventral_image())
