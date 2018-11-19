import RPi.GPIO as GPIO
from time import sleep
import Adafruit_PCA9685
import xbox
import picamera
from time import sleep
from PIL import Image
import io


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
    
def XXX():
    GPIO.output(LeftMotorEn, 0)
    LeftPWM.ChangeDutyCycle(0)
    GPIO.output(RightMotorEn, 0)
    RightPWM.ChangeDutyCycle(0)



def save_image_with_direction(stream, direction,n):
    """Save image"""
    stream.seek(0)
    image = Image.open(stream)
    image.save('Data/i,{},{},{}.jpeg' .format(n,int(direction[0]),direction[1]), format="JPEG")


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
        camera.framerate = 90
        camera.resolution = (640, 320) # Change for perfect image size!
        sleep(2)
        pwm_l,pwm_r,pwm = get_PWM()
        start_PWM(pwm_l, pwm_r)
        joy = xbox.Joystick()
        n = 0
        while True:
              if joy.leftStick():
                    # Transforming input x,y in acc and angle parameter - steeringfunction : angle_parm = 90*x+410
                    # steering min 320 steering max = 510
                    x,y = joy.leftStick()
                    acceleration = 100 * y*abs(y)
                    angle_param = int(80*x + 410)
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
                
                
              if joy.A():
                  
                  print("Paused!")
                  RearMotors(pwm_l, pwm_r,0,0,0,0,0,0)
                  steer_control(410,pwm)
              if joy.B():
                  print("B pressed! Goodbye!")
                            #set steering to netral and closing everything
                  steer_control(410,pwm)
                  control_sequence = False
                  joy.close()
                  GPIO.cleanup()
                  camera.close()
                  return False
                
              stream = io.BytesIO()
              camera.capture(stream, format = "jpeg", use_video_port = True)
              save_image_with_direction(stream, control_order,n)
              n+=1
              stream.flush()

                


def main():
    setup_pins()
    steering()
    
if __name__ == "__main__":
    main()
