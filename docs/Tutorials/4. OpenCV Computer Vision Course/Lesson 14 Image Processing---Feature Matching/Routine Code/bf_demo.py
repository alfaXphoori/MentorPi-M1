import cv2
 
img1 = cv2.imread('test.jpg')
img2=cv2.imread('test1.jpg')
 
# initialize ORB feature detector
orb = cv2.ORB_create()
 
# detect feature and descriptor 
kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)
 
# Create a brute force (BF) matcher
bf = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)
 
# match descriptor
matches = bf.match(des1,des2)
 
# draw ten matched descriptors 
img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], None, flags=2)
 
cv2.imshow("show",img3)
cv2.waitKey()
cv2.destroyAllWindows()