from PIL import Image as pil_image
import numpy as np
import cv2

img = pil_image.open('/local_home/rakeshs/880/dataset/images/0/map_00031.png')
x = np.asarray(img)

cv2.imshow("window", x[:, :, 2])
cv2.waitKey()