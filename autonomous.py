import RPi.GPIO as GPIO
from time import sleep
import Adafruit_PCA9685
import xbox
import picamera
from time import sleep
from PIL import Image
import io
import cv2
import numpy as np
from keras.models import load_model

def setup_pins():
    global LeftMotor1
    LeftMotor1 = 17 #Schwarz IN1
    global LeftMotor2
    LeftMotor2 = 18 #Weiss IN2
    global LeftMotorEn
    LeftMotorEn = 4 #grau EN1
    global RightMotor1
    RightMotor1 = 22 #grau IN3
    global RightMotor2
    RightMotor2 = 27 #lila IN4
    global RightMotorEn
    RightMotorEn = 25 #weiss En2
    #________________________
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(LeftMotor1, GPIO.OUT)
    GPIO.setup(LeftMotor2, GPIO.OUT)
    GPIO.setup(LeftMotorEn, GPIO.OUT)
    GPIO.setup(RightMotor1, GPIO.OUT)
    GPIO.setup(RightMotor2, GPIO.OUT)
    GPIO.setup(RightMotorEn, GPIO.OUT)
    
    control_order = (0,0)
    control_sequence = True
def get_PWM():
    LeftPWM = GPIO.PWM(LeftMotorEn, 60) # Hz anpassen
    RightPWM = GPIO.PWM(RightMotorEn, 60) # Hz anpassen
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(60)
    return LeftPWM,RightPWM,pwm

def start_PWM(LeftPWM,RightPWM):
    LeftPWM.start(0)
    RightPWM.start(0)
    




def RearMotors(LeftPWM,RightPWM,l1,l2, l, r1,r2,r):
#print("Duties: {},{},{}".format(l1,l2,l))
    LeftPWM.ChangeDutyCycle(l)
    GPIO.output(LeftMotor1, l1)
    GPIO.output(LeftMotor2, l2)
    RightPWM.ChangeDutyCycle(r)
    GPIO.output(RightMotor1, r1)
    GPIO.output(RightMotor2, r2)

def steer_control(angle,pwm):
    print("Angle_param: {}".format(angle))
    pwm.set_pwm(0,0, angle)
        


def steering():
    
    with picamera.PiCamera() as camera:
        camera.vflip = True
        camera.hflip = True
        camera.framerate = 90     # adapt frame rate since prediction might be too slow for realtime
        camera.resolution = (640, 320) # Change for perfect image size!
        sleep(2)
        pwm_l,pwm_r,pwm = get_PWM()
        start_PWM(pwm_l, pwm_r)
        model = load_model("Ns_Mini_model.h5")
        print("\nModel geladen, Auto ist bereit zum fahren!\n")
        iput = input("Drück [y/n] zum starten")
        if iput == "y":
            pass
        else:
            print("Abbruch!")
            exit()
        while True:

            stream = io.BytesIO()
            camera.capture(stream, format = "jpeg", use_video_port = True)
            #preprocessing of the image
            frame = cv2.imdecode(np.fromstring(stream.getvalue(), dtype =np.uint8),1)
            frame = cv2.resize(frame, (64,64)) #resizing
            norm_img = np.array(frame, dtype = "float")/255. #normalizing
            # expanddim:
            norm_img = np.expand_dims(norm_img, axis= 0)
            #print("img_size", norm_img.shape)
            
            #model predict:
            
            pred = model.predict(norm_img)
            steering = round(pred[0][1],1)
            acceleration = round(pred[0][0],1)
            #transforming in parameters for Servomotor
            acceleration = 100 * acceleration*abs(acceleration)
            acceleration = 32 # slow the car down a bit
            angle_param = int(80*steering + 410)
            control_order = (acceleration, angle_param)

            print("Y-Axis: {}".format(acceleration))
            # Forward - Right
            if acceleration > 0 and angle_param > 410:
                RearMotors(pwm_l, pwm_r,1, 0, acceleration, 1, 0, acceleration)
                steer_control(angle_param,pwm)
                                
            # Forward - Left
            if acceleration > 0 and angle_param < 410:
                RearMotors(pwm_l, pwm_r,1, 0, acceleration, 1, 0, acceleration)
                steer_control(angle_param,pwm)
                                
            # Backward - Right
            if acceleration < 0 and angle_param > 410:
            #Powering backwards attention power hast to be positiv _ output is negative-> *-1
                RearMotors(pwm_l, pwm_r,0, 1, -1*acceleration,0,1,-1*acceleration)
                steer_control(angle_param,pwm)
                                
            # Backward - Right
            if acceleration < 0 and angle_param < 410:
            #Powering backwards attention power hast to be positiv _ output is negative-> *-1
                RearMotors(pwm_l, pwm_r,0, 1, -1*acceleration,0,1,-1*acceleration)
                steer_control(angle_param,pwm)
                                
            if acceleration == 0:
                RearMotors(pwm_l, pwm_r,0,0,0,0,0,0)
                
            #Abbruch Kriterium    
            #if Abbruch_kriterium:   
            #    print("Paused!")
            #    RearMotors(pwm_l, pwm_r,0,0,0,0,0,0)
            #    steer_control(410,pwm)
            #    break
            stream.flush()
        GPIO.cleanup()
        camera.close()
                  
                

        

                


def main():
    #loading the model:
    
    setup_pins()
    steering()
    
if __name__ == "__main__":
    main()
