# Autonomous-Robocar


[image1]: ./Imgs/DSC_0024_2.JPG "purple_rain"
[image2]: ./Imgs/Schaltplan_v1-1.png "Schaltplan_v1"
[image3]: ./Imgs/IMG_20181115_120428.jpg "Auto_camtower"
[image4]: ./Imgs/Circuit2.png "circuit2"
[image5]: ./Imgs/Predictions.png "Recovery Image"
[image6]: ./Imgs/Predictions_arrow.png "Normal Image"
[image7]: ./Imgs/learning_curve.png "Flipped Image"


![alt text][image3]

## Schaltplan v1
![alt text][image2]


![alt text][image1]


## Needed Libraries
- [Tensorflow and Keras](https://medium.com/@abhizcc/installing-latest-tensor-flow-and-keras-on-raspberry-pi-aac7dbf95f2)
- sudo pip install adafruit-pca9685
- sudo apt-get install xboxdrv
- [OpenCV](https://www.alatortsev.com/2018/04/27/installing-opencv-on-raspberry-pi-3-b/)

## Enable in the Raspberry Pi Config:
- I2C


## Collecting training data

- for collecting training data run ```sudo python complete_control.py```


## Training the network

![alt text][image7]

- training the CNN with the gathered data to predict steering angle and acceleration

## Predictions and further visualizations

- after successfully predicting steering angle and acceleration I decided to write the predictions directly on the image

![alt text][image5]

- further implemented an arrow that visualizes the direction the network predicted to steer. The length of the arrow as well symbolizes the intensity of acceleration.

![alt text][image6]


### Updates and Changes:

- trained a smaller model for predictions because loading and calc with the previous model was too calculation costly.
Therefore I trained another - smaller model- and increased the size from 331 MB to 1.4 MB. While the accuracy still is up to 92% compared to 97% for the bigger model! huge success in my eyes!






### final video output:

[Video](https://www.youtube.com/watch?v=O1CGPuIWheo)

## TODO: 
- add ultrasonic 
- optimize steering and acceleration adaption
