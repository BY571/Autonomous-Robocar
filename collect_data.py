import RPi.GPIO as GPIO
from time import sleep
import Adafruit_PCA9685
import xbox
import picamera
import thread
import csv

class Controller:

    LeftMotor1 = 17 #Schwarz IN1
    LeftMotor2 = 18 #Weiss IN2
    LeftMotorEn = 4 #grau EN1
    RightMotor1 = 22 #grau IN3
    RightMotor2 = 27 #lila IN4
    RightMotorEn = 25 #weiss En2

    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.LeftMotor1, GPIO.OUT)
        GPIO.setup(self.LeftMotor2, GPIO.OUT)
        GPIO.setup(self.LeftMotorEn, GPIO.OUT)
        GPIO.output(self.LeftMotorEn, 0)
        self.LeftPWM = GPIO.PWM(self.LeftMotorEn, 60) # Hz anpassen
        self.LeftPWM.start(0)
        self.LeftPWM.ChangeDutyCycle(0)
        GPIO.setup(self.RightMotor1, GPIO.OUT)
        GPIO.setup(self.RightMotor2, GPIO.OUT)
        GPIO.setup(self.RightMotorEn, GPIO.OUT)
        GPIO.output(self.RightMotorEn, 0)
        self.RightPWM = GPIO.PWM(self.RightMotorEn, 60) # Hz anpassen
        self.RightPWM.start(0)
        self.RightPWM.ChangeDutyCycle(0)
        #Steering
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
    	
    	self.control_order = (0,0)
    	self.control_sequence = True

    def RearMotors(self,l1,l2, l, r1,r2,r):
        #print("Duties: {},{},{}".format(l1,l2,l))
        self.LeftPWM.ChangeDutyCycle(l)
        GPIO.output(self.LeftMotor1, l1)
        GPIO.output(self.LeftMotor2, l2)
        self.RightPWM.ChangeDutyCycle(r)
        GPIO.output(self.RightMotor1, r1)
        GPIO.output(self.RightMotor2, r2)

    def steer_control(self, angle):
        print("Angle_param: {}".format(angle))
        self.pwm.set_pwm(0,0, angle)
        
        

    def steering(self):
        joy = xbox.Joystick()
        while True:
	        if joy.leftStick():
	                # Transforming input x,y in acc and angle parameter - steeringfunction : angle_parm = 90*x+410
	                # steering min 320 steering max = 510
                    x,y = joy.leftStick()
                    acceleration = 100 * y*abs(y)
	            angle_param = int(80*x + 410)
	            self.control_order = (acceleration, angle_param)
	            print("Y-Axis: {}".format(acceleration))
	                # Forward - Right
	            if acceleration > 0 and angle_param > 410:
	            	self.RearMotors(1, 0, acceleration, 1, 0, acceleration)
	                self.steer_control(angle_param)
	                    
	                # Forward - Left
	            if acceleration > 0 and angle_param < 410:
	                self.RearMotors(1, 0, acceleration, 1, 0, acceleration)
	            	self.steer_control(angle_param)
	                    
	                # Backward - Right
	            if acceleration < 0 and angle_param > 410:
	                    #Powering backwards attention power hast to be positiv _ output is negative-> *-1
	                self.RearMotors(0, 1, -1*acceleration,0,1,-1*acceleration)
	                self.steer_control(angle_param)
	                    
	                # Backward - Right
	            if acceleration < 0 and angle_param < 410:
	                    #Powering backwards attention power hast to be positiv _ output is negative-> *-1
	                self.RearMotors(0, 1, -1*acceleration,0,1,-1*acceleration)
	                self.steer_control(angle_param)
	                    
	            if acceleration == 0:
	                self.RearMotors(0,0,0,0,0,0)

	        if joy.A():
	            print("Paused!")
	            self.RearMotors(0,0,0,0,0,0)
	            self.steer_control(410)
	        if joy.B():
	            print("B pressed! Goodbye!")
	                #set steering to netral and closing everything
	            self.steer_control(410)
	            self.control_sequence = False
	            joy.close()
	            GPIO.cleanup()
	            return False
	

class Camera:
    imgsPath = "Data/Img/"

    def __init__(self):
        self.camera = picamera.PiCamera()
        self.camera.vflip = True
        self.camera.hflip = True
        #self.camera.iso = 1600
        self.camera.framerate = 90
        self.camera.shutter_speed = 2000    #800
        self.camera.resolution = (320, 160)
        self.camera.start_preview()
        sleep(2)
        #self.stream = picamera.array.PiRGBArray(self.camera)
        
    def make_img(self, nu):
        self.camera.capture("Data/Imgs/{}.jpeg".format(nu), use_video_port = True)
  
    def close_cam(self):
        self.camera.stop_preview()
        self.camera.close()

def main():
	try:
            i = 0
            history = []
            carCtrl = Controller()
            camCtrl = Camera()
            thread.start_new_thread(carCtrl.steering,())
            while True:
        
                
                #camCtrl.save_img(camCtrl.capture())
                camCtrl.make_img(i)
                history.append(carCtrl.control_order)
                i += 1
                
                # check if stop capturing
                if carCtrl.control_sequence == False:
                    print("Checked if control_sequence is still alive :: No!")
                    break
        
            with open("Data/history.csv","w") as csvfile:
                filecsv = csv.writer(csvfile, delimiter = ",")
                for command in history:
                    filecsv.writerow([command])
        
        except OSError as err:
            print(err)
            pass
            
        finally:
            camCtrl.close_cam()
            




if __name__ == '__main__':
	main()

    
