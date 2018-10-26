from __future__ import division
import Adafruit_PCA9685
import xbox

class Controller:

    def __init__(self):
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        self.servo_min = 320 
        self.servo_max = 500
    
    def steer_control(self, angle):
        print("Angle_param: {}".format(angle))
        self.pwm.set_pwm(0,0, angle)

    def steering(self):
        joy = xbox.Joystick()
        while True:
            if joy.leftX():
                #an dem Wert fr die Lenkung spielen.. was macht sinn wo laesst sich das auto gut steuern
                x_value =  joy.leftX()
                print("X-Axis: {}".format(x_value))
                if x_value > 0:
                    #Powering steering right
                    angle_param = int(80*x_value + 410) # steeringfunction: 90x+410
                    self.steer_control(angle_param)

                if x_value < 0:
                    #Powering steering
                    angle_param = int(80*x_value + 410) # steeringfunction: 90x+410
                    self.steer_control(angle_param)
                    
                if x_value == 0:
                    self.steer_control(410)

                
            if joy.A():
                print("Paused!")
                self.steer_control(410)
            if joy.B():
                print("B pressed! Goodbye!")
                joy.close()
                
                break



if __name__ == '__main__':
    try:
        carCtrl = Controller()
        carCtrl.steering()
        
    except OSError as err:
        print(err)
        pass
