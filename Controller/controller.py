import RPi.GPIO as GPIO
from time import sleep
import Adafruit_PCA9685
import xbox

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
