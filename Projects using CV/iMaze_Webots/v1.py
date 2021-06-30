# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, Camera
import numpy as np
import cv2
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

max_speed = 6.28

prox_sensor = []
for i in range(8):
    name = 'ps' + str(i)
    prox_sensor.append(robot.getDevice(name))
    prox_sensor[i].enable(timestep)
    
camera = robot.getDevice('camera')
camera.enable(timestep)


def moveForward(speed):
    if(speed>100):
        speed = 100
    speed = (max_speed/100.0)*speed
    for _ in range(10):
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed)
# Main loop:

def frontIrReading():
    return math.floor(prox_sensor[0].getValue()+prox_sensor[7].getValue())
    
def leftIrReading():
    return math.floor(prox_sensor[5].getValue())
    
def rightIrReading():
    return math.floor(prox_sensor[2].getValue())
    
def forward_backward(speed):
    if(speed>100):
        speed = 100
    speed = (max_speed/100.0)*speed
    for _ in range(10):
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(-speed)
    
    
    
def rotate(speed):
    if(speed>100):
        speed = 100
    speed = (max_speed/100.0)*speed
    for _ in range(10):
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(speed)
        
def turn_right(speed):
    if(speed>100):
        speed = 100
    speed = (max_speed/100.0)*speed
    for _ in range(10):
        left_motor.setVelocity(speed)
        right_motor.setVelocity(-speed)
        
def turn_left(speed):
    if(speed>100):
        speed = 100
    speed = (max_speed/100.0)*speed
    for _ in range(10):
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(speed)
        
def find_green(frame_g,found):
    flag=0
    if not found:
        green_lower = np.array([25, 52, 72], np.uint8)
        green_upper = np.array([102, 255, 255], np.uint8)
        median_g=cv2.medianBlur(hsv, 5)
        green_mask = cv2.inRange(median_g, green_lower, green_upper)
        cg, gg = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    
        for g in cg:
            ag = cv2.contourArea(g)
            if ag > 500:
            
                x, y, w, h = cv2.boundingRect(g)
                cv2.rectangle(green_mask, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.drawContours(green_mask, g, -1, (0, 255, 0), 2)
                res_g = cv2.bitwise_and(frame_g, frame_g, mask=green_mask)
                _, th = cv2.threshold(res_g, 0, 255, cv2.THRESH_BINARY)
                k = np.ones((3, 3), np.uint8)
                d = cv2.dilate(th, k, iterations=5)
                #cv2.imshow("green", d)
                M=cv2.moments(g)
                cx=int(M['m10']/M['m00'])
                print("cx=",cx)
                print("green area=",ag)
                if cx>=127 and cx<=200:
                    if ag>65000 and frontIrReading()>150:
                        found=True
                        print("green was found successfully")
                        turn_right(100)
                    else:   
                                        
                        moveForward(100)
                        print("forward motion activated")
                        print("front", frontIrReading())
                else:
                    turn_left(30)
    return found
   
       
def find_blue(frame_b,found):
    flag=0
    if not found:
        blue_lower = np.array([110, 150, 50], np.uint8)
        blue_upper = np.array([130, 255, 255], np.uint8)
        median_b=cv2.medianBlur(hsv, 5)
        
        blue_mask = cv2.inRange(median_b, blue_lower, blue_upper)

        cb, gb = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        for b in cb:
            ag = cv2.contourArea(b)
            if ag > 500:
                x, y, w, h = cv2.boundingRect(b)
                cv2.rectangle(blue_mask, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.drawContours(blue_mask, b, -1, (0, 0, 255), 2)
                res_b= cv2.bitwise_and(frame_b, frame_b, mask=blue_mask)
                _, th = cv2.threshold(res_b, 0, 255, cv2.THRESH_BINARY)
                k = np.ones((3, 3), np.uint8)
                d = cv2.dilate(th, k, iterations=5)
                #cv2.imshow("blue", d)
                M=cv2.moments(b)
                cx=int(M['m10']/M['m00'])
                print("cx(blue)=",cx)
                print("blue area=",ag)
                if ag<65000:       
                    if cx>=120 and cx<=200:
                        print("approaching blue")
                        moveForward(100)
                    elif frontIrReading()>146 or (cx<=140 or cx>=180):
                        turn_right(30)
                        print("searching for blue")
                else:
                    found=True
                    print("blue found successfully")
    return found  
       

    
def stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
    
flagg=0
flagb=0
flagr=0
flagp=0
flag=0
found_green=False
found_blue=False

# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    rgb = camera.getImageArray()
    camera.saveImage('cap.jpg',100)
    frame=cv2.imread('cap.jpg',1)
    frame=cv2.resize(frame,(400,400))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_r=frame
    frame_g=frame
    frame_b=frame
    frame_p=frame
    
    found_green=find_green(frame,found_green)
    if found_green:
        found_blue=find_blue(frame,found_blue)
        
    if found_blue:
        break
                
         
    red_lower = np.array([165, 48, 0], np.uint8)
    red_upper = np.array([179, 255, 255], np.uint8)
    median_r = cv2.medianBlur(hsv, 1)
    red_mask = cv2.inRange(median_r, red_lower, red_upper)    
    cr,gr = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    for c in cr:
        ar=cv2.contourArea(c)
        if ar>500:
          x, y, w, h =cv2.boundingRect(c)
          cv2.rectangle(red_mask,(x,y),(x+w,y+h),(0,0,255),2)
          cv2.drawContours(red_mask, c, -1,(255,0,0),2)
          res_r = cv2.bitwise_and(frame_r, frame_r, mask=red_mask)
          _, th =cv2.threshold(res_r, 0,255,cv2.THRESH_BINARY)
          k=np.ones((7,7),np.uint8)
          d=cv2.dilate(th, k, iterations=5)
          #cv2.imshow("red",d)            
            
            
    purple_lower = np.array([80, 100, 10], np.uint8)
    purple_upper = np.array([120,255, 255], np.uint8)
    median_p=cv2.medianBlur(hsv, 5)

    purple_mask = cv2.inRange(median_p, purple_lower, purple_upper)

    cp, gp = cv2.findContours(purple_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    for p in cp:
        ap = cv2.contourArea(p)
        if ap > 100:
            x, y, w, h = cv2.boundingRect(p)
            cv2.rectangle(purple_mask, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.drawContours(purple_mask, p, -1, (0, 0, 255), 2)
            res_p= cv2.bitwise_and(frame_p, frame_p, mask=purple_mask)
            _, th = cv2.threshold(res_p, 0, 255, cv2.THRESH_BINARY)
            k = np.ones((3, 3), np.uint8)
            d = cv2.dilate(th, k, iterations=5)
            #cv2.imshow("violet", d)
    print("front=",frontIrReading())
    #if flagg==0:
        #rotate(30)
    #if flagb!=-1 and found_green:
        #if frontIrReading()>190:
            #rotate(20)
        #else:
            #turn(80)
            #print("turn")
    #elif flagb==0:
        #rotate(50)
    cv2.waitKey(1)
    
    #Using 'img' as image object do whatever processing you want.
   
   

    # Process sensor data here.
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.