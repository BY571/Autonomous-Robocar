import picamera
from time import sleep

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
