from controller import Robot, Motor, Gyro, GPS, Camera, Compass, Keyboard, LED, InertialUnit, DistanceSensor
import math
import cv2
import numpy as np


SIGN = lambda x: int(x>0) - int(x<0)
CLAMP = lambda value, low, high : min(high, max(value, low))

# Display barcode and QR code location
def display(im, bbox):
    n = len(bbox)
    for j in range(n):
        cv2.line(im, tuple(bbox[j][0]), tuple(bbox[ (j+1) % n][0]), (255,0,0), 3)

    # Display results
    cv2.imshow("Results", im)
def qrCode(img):
            qrDecoder = cv2.QRCodeDetector()
    
    
            data,bbox,rectifiedImage = qrDecoder.detectAndDecode(img)
            if len(data)>0:
                print("Decoded Data : {}".format(data))
                display(inputImage, bbox)
                rectifiedImage = np.uint8(rectifiedImage);
                cv2.imshow("Rectified QRCode", rectifiedImage);
            else:
                print("QR Code not detected")
                cv2.imshow("Results", img)
            
            cv2.waitKey(0)
            cv2.destroyAllWindows() 
class Drone:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.camera = self.robot.getDevice('camera');
        self.camera.enable(self.timestep)
        # front_left_led = robot.getDevice("front left led");
        # front_right_led = robot.getDevice("front right led");
        self.imu = self.robot.getDevice("inertial unit");
        self.imu.enable(self.timestep);
        self.gps = self.robot.getDevice("gps");
        self.gps.enable(self.timestep);
        self.compass = self.robot.getDevice("compass");
        self.compass.enable(self.timestep);
        self.gyro = self.robot.getDevice("gyro");
        self.gyro.enable(self.timestep);
        self.ds_front=self.robot.getDevice("ds_front")
        self.ds_front.enable(self.timestep)
        self.ds_right=self.robot.getDevice("ds_right")
        self.ds_right.enable(self.timestep)
        self.ds_left=self.robot.getDevice("ds_left")
        self.ds_left.enable(self.timestep)
        self.ds_bottom=self.robot.getDevice("ds_bottom")
        self.ds_bottom.enable(self.timestep)
       
        # keyboard = Keyboard();
        # keyboard.enable(timestep)
        self.camera_roll_motor = self.robot.getDevice('camera roll');
        self.camera_pitch_motor = self.robot.getDevice('camera pitch');

        self.front_left_motor = self.robot.getDevice("front left propeller");
        self.front_right_motor = self.robot.getDevice("front right propeller");
        self.rear_left_motor = self.robot.getDevice("rear left propeller");
        self.rear_right_motor = self.robot.getDevice("rear right propeller");
        self.motors = [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor];

        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)

        self.k_vertical_thrust = 68.5
        self.k_vertical_offset = 0.6 
        self.k_vertical_p = 3
        self.k_roll_p = 50.0
        self.k_pitch_p = 40.0
        self.k_yaw_p = 0

        self.target_altitude = 1.0
        
        self.target = [38.85, -229.23]

    def move(self,command,intensity):
        roll = self.imu.getRollPitchYaw()[0] + math.pi / 2.0
        pitch = self.imu.getRollPitchYaw()[1]
        altitude = self.gps.getValues()[1]
        roll_acceleration = self.gyro.getValues()[0]
        pitch_acceleration = self.gyro.getValues()[1]
        yaw_acceleration = self.gyro.getValues()[2]

        # led_state = int(time) % 2
        # front_left_led.set(led_state)
        # front_right_led.set(int(not led_state))
        
        self.camera_roll_motor.setPosition(-0.115 * roll_acceleration)
        self.camera_pitch_motor.setPosition(-0.1 * pitch_acceleration)
        
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0

        if(command=='forward'):
            pitch_disturbance = intensity  #2.0
        elif(command=='backward'):
            pitch_disturbance = -intensity #-2.0
        elif(command=='right'):
            yaw_disturbance = intensity  #1.3
        elif(command=='left'):
            yaw_disturbance = -intensity  #-1.3
        elif(command=='sRight'):
            roll_disturbance = intensity  #-1.0
        elif(command=='sLeft'):
            roll_disturbance = intensity  #1.0
        elif(command=='up'): 
            self.target_altitude += intensity  #0.05
        elif(command=='down'):
            self.target_altitude -= intensity  #0.05

        roll_input = self.k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
        pitch_input = self.k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance
        yaw_input = self.k_yaw_p * CLAMP(roll, -1.0, 1.0)+ yaw_disturbance + yaw_acceleration
        clamped_difference_altitude = CLAMP(self.target_altitude - altitude + self.k_vertical_offset, -1.0, 1.0)
        vertical_input = self.k_vertical_p * pow(clamped_difference_altitude, 3.0)

        
        front_left_motor_input = self.k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
        front_right_motor_input = self.k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
        rear_left_motor_input = self.k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
        rear_right_motor_input = self.k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
        self.front_left_motor.setVelocity(front_left_motor_input)
        self.front_right_motor.setVelocity(-front_right_motor_input)
        self.rear_left_motor.setVelocity(-rear_left_motor_input)
        self.rear_right_motor.setVelocity(rear_right_motor_input)

    def get_image(self):
        image=self.camera.getImageArray()
        image=np.array(image,dtype=np.uint8)
        return image
        
    def get_x(self):
        x  = self.gps.getValues()[0]
        return x
        
    def get_y(self):
        y = self.gps.getValues()[1]
        return y
    def get_z(self):
        z = self.gps.getValues()[2]
        return z
        
    def dist(self, target):
        d = (target[0]-self.gps.getValues()[0])**2 + (target[1]-self.gps.getValues()[2])**2
        return math.sqrt(d)    
    def vector(self, target):
        # v = (target[0]-self.gps.getValues()[0])/(target[2]-self.gps.getValues()[2])
        angle = math.atan2(target[0]-self.gps.getValues()[0],target[1]-self.gps.getValues()[2])*180/3.14 
        # if(angle < 0):
            # angle = angle + 360
        return angle   
             
    def qrscan(self, img):
        from pyzbar.pyzbar import decode
        decoded = decode(img)
        if(decoded):
            data  = decoded[0].data.decode("utf-8")
    
            for i in range(len(data)):
            	if(data[i]==','):
            	  pos = i
            x = float(data[1:pos])
            y = float(data[pos+1:len(data)-1])
            target = [x, y]   
            return target     
        return None
        
        
                
        
    
    def goto(self, target, i):
        c=0
        final_dir = self.vector(target)
        final_yaw = self.imu.getRollPitchYaw()[2]*180/3.14 + 90
        if(final_dir < 0 ):
            final_dir = final_dir + 360
        if( final_yaw < 0):
            final_yaw += 360 
        # print('final yaw' , final_yaw)
        # print('final_ dir' , final_dir) 
        
        final_diff = final_yaw - final_dir  
        # if final_diff<0 :
            # final_diff += 360
        # if final_diff>10 and final_diff<180:
            # self.move('right', 0.5)
        # elif final_diff>180 and final_diff<350:
            # self.move('left', 0.5)    
        # else:
            # self.move('right', 0)  
            # self.move('left', 0)
            # d = self.dist(target)
            # if d > 1000:
                # self.move('forward', 2)  
            # else:
                # self.move('forward', 0.5)
            # print('moving forward')
                
        print('final_diff' , int(final_diff))

        
        d = self.dist(target)
        bottom_ds = self.ds_bottom.getValue()
        front_ds = self.ds_front.getValue() + self.ds_left.getValue() + self.ds_right.getValue()
        if(bottom_ds>650 or i < 3000):
                self.move('up',0.015)
                print('moving up')
    
        elif(bottom_ds<=450 or i>=2000):
                if(bottom_ds>=450):
                   self.move('up',0.2)
                   print('emergeny liftoff')
        if(self.get_y()>60):
            self.move('up', 0)           
        
        if(front_ds>201):
            self.move('up',0.02)
            self.move('backward',1.5)
            print('obstacle ahead')
            
        else:
            if final_diff<0 :
                final_diff += 360
            if final_diff>5 and final_diff<180:
                self.move('right', 0.5)
            elif final_diff>180 and final_diff<355:
                self.move('left', 0.5)    
            else:
                self.move('right', 0)  
                self.move('left', 0)
                
                if d > 20:
                    self.move('forward', 1.5)  
                else:
                    self.move('forward', 0.5)
            print('moving forward')
                
        # print('final_diff' , final_diff)

   
        if(d<1):
            if(self.qrscan(self.get_image())):
                
                self.target = self.qrscan(self.get_image())
              
            
            
                
            
              
     
drone=Drone()



i=0
c=0
while drone.robot.step(drone.timestep) != -1:
    

    x = drone.get_x()
    y = drone.get_y()
    z = drone.get_z()
    yaw = drone.imu.getRollPitchYaw()[2]
    bottom_ds = drone.ds_bottom.getValue()
    front_ds = drone.ds_front.getValue() + drone.ds_left.getValue() + drone.ds_right.getValue()
    
    i=i+1

 
    print("x=",int(x),' y=', int(y),' z=', int(z))
    print("dist :",int(drone.dist(drone.target)))

    print("bottom sensor reading=",bottom_ds)
    print("front sensor reading=", front_ds)
    drone.goto(drone.target,i)

  
cv2.destroyAllWindows()