import numpy as np
import cv2
import math
from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

max_speed = 6.27

prox_sensor = []
for i in range(8):
 name = 'ps' + str(i)
 prox_sensor.append(robot.getDevice(name))
 prox_sensor[i].enable(timestep)

while robot.step(timestep) != -1:
  for i in range(8):
     left_wall = prox_sensor[5].getValue()>80
     front_wall = prox_sensor[7].getValue()>80
     left_corner = prox_sensor[0].getValue()>80
     op = prox_sensor[2].getValue()<80



     left_speed = max_speed
     right_speed = max_speed

     if front_wall and left_corner:
        left_speed = max_speed
        right_speed =-max_speed
        
     if front_wall and op:
        left_speed=0
        right_speed=0
     
     
 
    

     left_motor.setVelocity(left_speed)
     right_motor.setVelocity(right_speed)


pass