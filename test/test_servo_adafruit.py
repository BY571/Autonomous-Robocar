from __future__ import division
import time
import Adafruit_PCA9685

# init Ada_PWM
pwm = Adafruit_PCA9685.PCA9685()
# min and max servo Pulse length
servo_min = 320 # min length out of 4096   -- 150
servo_max = 500 # max length out of 4096   -- 650

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000 # 1000.000 us per second
    pulse_length //=60 #60Hz
    print("{0} us per second".format(pulse_length))
    pulse_length //= 4096 #12 bit resolution
    print("{0} us per bit".format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)
    
pwm.set_pwm_freq(60)
print("Moving servo on channel 0! - press cltr-C to quit")
while True:
    # Move servo on channel 0 between extremes
    pwm.set_pwm(0,0, servo_min)
    time.sleep(1)
    pwm.set_pwm(0,0, servo_max)
    time.sleep(1)
    