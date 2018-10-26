import cv2
import os

image_folder = 'Data/Imgs'
video_name = 'video.mp4'

images = [img for img in os.listdir(image_folder) if img.endswith(".jpeg")]
#images = images["*.jpeg"].sort(key = int)
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

#Sort images by numbers 
images_index = []
for i in images:
    name,a = i.split(".")
    #print(name)
    images_index.append(name)
images_index.sort(key = int)
#print(images_index)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(video_name,fourcc , 20, (width,height))


for image in images_index:
    print(image)
    out.write(cv2.imread(os.path.join(image_folder, image+".jpeg")))

# Release everything if job is finished

out.release()
cv2.destroyAllWindows()