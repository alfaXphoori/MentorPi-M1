import numpy as np
import cv2 as cv

src = cv.imread('1.jpg')
# method  output the dimension directly
height, width = src.shape[:2]  # acquire the original dimension
res1 = cv.resize(src, (int(1.2*width), int(1.2*height)),interpolation=cv.INTER_CUBIC)
res2 = cv.resize(src, (int(0.6*width), int(0.6*height)),interpolation=cv.INTER_CUBIC)
cv.imshow("src", src)
cv.imshow("res1", res1)
cv.imshow("res2", res2)
print("src.shape=", src.shape)
print("res1.shape=", res1.shape)
print("res2.shape=", res2.shape)
cv.waitKey()
cv.destroyAllWindows()
