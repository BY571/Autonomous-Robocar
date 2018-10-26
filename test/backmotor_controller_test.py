import RPi.GPIO as GPIO
from time import sleep
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
    
    def RearMotors(self,l1,l2, l, r1,r2,r):
        #print("Duties: {},{},{}".format(l1,l2,l))
        self.LeftPWM.ChangeDutyCycle(l)
        GPIO.output(self.LeftMotor1, l1)
        GPIO.output(self.LeftMotor2, l2)
        self.RightPWM.ChangeDutyCycle(r)
        GPIO.output(self.RightMotor1, r1)
        GPIO.output(self.RightMotor2, r2)

    def steering(self):
        joy = xbox.Joystick()
        while True:
            #if joy.leftX():
                #an dem Wert fr die Lenkung spielen.. was macht sinn wo laesst sich das auto gut steuern
            #    steer = 40 * joy.leftX()*abs(joy.leftX())
            #    print("X-Axis: {}".format(steer))
            #    if steer >= 0:
                    #Powering steering right
            #        self.front(1, 0, steer)
                    #self.direction = 1

            #    if steer < 0:
                    #Powering steering
            #        self.front(0, 1, -1*steer)
                    #self.direction = 2
            #    else:
		    #pass
                
            if joy.leftY():
                # 50 - max power for acceleration (TUNING PARAMETER!)
                acceleration = 100 * joy.leftY()*abs(joy.leftY())
                print("Y-Axis: {}".format(acceleration))
                if acceleration > 0:
                    #Powering forward
                    self.RearMotors(1, 0, acceleration, 1, 0, acceleration)
                if acceleration < 0:
                    #Powering backwards attention power hast to be positiv _ output is negative-> *-1
                    self.RearMotors(0, 1, -1*acceleration,0,1,-1*acceleration)
                if acceleration == 0:
                    self.RearMotors(0,0,0,0,0,0)
                #else:
                #    self.RearMotors(0,0,0,0,0,0)
            if joy.A():
                print("Paused!")
                self.RearMotors(0,0,0,0,0,0)
            if joy.B():
                print("B pressed! Goodbye!")
                joy.close()
                GPIO.cleanup()
                break



if __name__ == '__main__':
    try:
        carCtrl = Controller()
        carCtrl.steering()
        
    except OSError as err:
        print(err)
        pass
