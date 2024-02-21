import cv2
import numpy as np
import glob
 
img_array = []
list_img = glob.glob('images/*.png')
list_img = sorted(list_img)

for filename in list_img:
    # print(filename)
    img = cv2.imread(filename)
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)
 
out = cv2.VideoWriter('results/results.mp4',cv2.VideoWriter_fourcc(*'MP4V'),20, size)
 
i = 0
while i < len(img_array):
    out.write(img_array[i])
    i += 1
out.release()