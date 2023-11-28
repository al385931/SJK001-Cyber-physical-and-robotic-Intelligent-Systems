from GUI import GUI
from HAL import HAL
# Enter sequential code!

VICTIMS_X = 30
VICTIMS_Y = -40
BOAT_X = 0
BOAT_Y = 0

x_vel = 0.25
angle = 0.6
iterations = 0
spiral_iterations = 300
landin_margin = 0.07

#Returns the actual position of the drone as a numpy array [x, y, z], in m.
x_pos = HAL.get_position()[0]
y_pos = HAL.get_position()[1]

#Takeoff at the current location, to the given height
HAL.takeoff(3)

while not ((VICTIMS_X -1 < x_pos) and (x_pos < VICTIMS_X +1 ) and (VICTIMS_Y -1 < y_pos) and (y_pos < VICTIMS_Y +1 )):
	GUI.showImage(HAL.get_frontal_image())
	GUI.showLeftImage(HAL.get_ventral_image())
	x_pos = HAL.get_position()[0]
	y_pos = HAL.get_position()[1]
	#Commands the position (x,y,z) of the drone, in m and the yaw angle (az) (in rad) taking as reference the first takeoff point (map frame)
	HAL.set_cmd_pos(VICTIMS_X, VICTIMS_Y, 3, angle)
  #time.sleep(0.01)
 
while True:
	GUI.showImage(HAL.get_frontal_image())
	GUI.showLeftImage(HAL.get_ventral_image())
