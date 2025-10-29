import cv2
import numpy as np
import matplotlib.pyplot as plt
import os


for filename in os.listdir('/home/marilene/github/Biomimicry_group6/Vision'):
    if filename.endswith(".jpg"):  
        image_path = os.path.join('/home/marilene/github/Biomimicry_group6/Vision', 'sand_pictures') 
        print("Processing:", image_path)

        image = cv2.imread(image_path)








h, w = grayscale(image).shape 

margin_y = h // 20
margin_x = w // 3

start_y = margin_y
end_y = h - margin_y
start_x = margin_x
end_x = w - margin_x

cropped = gray[start_y:end_y, start_x:end_x]

cv2.imshow("Cropped left-right", cropped)
cv2.waitKey(0)
cv2.destroyAllWindows()
