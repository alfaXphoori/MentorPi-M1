import cv2
import numpy as np

img = cv2.imread('1.jpg')
rows, cols, ch = img.shape
# rotate the center  rotation angle  scale factor
M = cv2.getRotationMatrix2D(((cols-1) / 2.0,(rows-1)/2.0), 90,1)
# original picture  convert matrix  output the image center
dst = cv2.warpAffine(img, M, (cols, rows))

cv2.imshow('img', img)
cv2.imshow('dst', dst)
cv2.waitKey(0)
cv2.destroyAllWindows()
