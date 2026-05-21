import cv2
import numpy as np
 
src = cv2.imread("img1.jpg")
src = cv2.resize(src, (int(src.shape[1] / 2), int(src.shape[0] / 2)))
 
GRAY = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
Lab = cv2.cvtColor(src, cv2.COLOR_BGR2LAB)
YCrCb = cv2.cvtColor(src, cv2.COLOR_BGR2YCrCb)
HSV = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
 
cv2.imshow("src", src)
cv2.imshow("GRAY", GRAY)
cv2.imshow("Lab", Lab)
cv2.imshow("YCrCb", YCrCb)
cv2.imshow("HSV", HSV)

cv2.waitKey(0)
cv2.destroyAllWindows()