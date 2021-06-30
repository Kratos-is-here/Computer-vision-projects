import cv2
import numpy as np
import math
import pytesseract
pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'
frame = cv2.imread('first in bgr.jpeg')
frame=cv2.resize(frame,(400,400))
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
green_lower = np.array([25, 52, 72], np.uint8)
green_upper = np.array([102, 255, 255], np.uint8)
median_g = cv2.medianBlur(hsv, 7)

green_mask = cv2.inRange(median_g, green_lower, green_upper)

green_mask = cv2.erode(green_mask, (9, 9), iterations=5)

cv2.imshow('dilated green', green_mask)

corners = cv2.goodFeaturesToTrack(green_mask, 4, 0.01, 100)

corners = np.int0(corners)

edges=np.zeros((4,2))
i = 0
xs = 0
ys = 0
for corner in corners:
    x, y = corner.ravel()
    edges[i, :]=[x, y]
    i += 1
    xs += x
    ys +=y
theta = math.degrees(math.atan((edges[0, 1]-edges[1, 1])/(edges[0, 0]-edges[1, 0])))
center = (xs / 4, ys / 4)
Matrix = cv2.getRotationMatrix2D(center, theta, 1.0)
frame = cv2.warpAffine(frame, Matrix, (400, 400))
corners1 = cv2.goodFeaturesToTrack(green_mask, 4, 0.01, 100)
corners1 = np.int0(corners1)
print(np.shape(corners1))
edges1=np.zeros((4,2))
i=0
for corner1 in corners1:
    x, y = corner1.ravel()
    edges1[i,:]=[x,y]
    i+=1
print(edges1)
x=int(edges1[0,0])
y=int(edges1[0,1])
w= (int(edges1[1,0]-edges1[0,0]))
h= (int(edges1[2,1]-edges1[0,1]))
a=min(x,x+w)
b=max(x,x+w)
c=min(y,y+h)
d=max(y,y+h)
crop_img = frame[c:d, a:b]
center = (crop_img.shape[0] / 2, crop_img.shape[1] / 2)
Matrix1 = cv2.getRotationMatrix2D(center, 90, 1.0)
Matrix2 = cv2.getRotationMatrix2D(center, 180, 1.0)
Matrix3 = cv2.getRotationMatrix2D(center, 270, 1.0)
crop_img1 = cv2.warpAffine(crop_img, Matrix1, (crop_img.shape[0],crop_img.shape[1]))
crop_img2 = cv2.warpAffine(crop_img, Matrix2, (crop_img.shape[0],crop_img.shape[1]))
crop_img3 = cv2.warpAffine(crop_img, Matrix3, (crop_img.shape[0],crop_img.shape[1]))
cv2.imshow("cropped0", crop_img)
cv2.imshow("cropped1", crop_img1)
cv2.imshow("cropped2", crop_img2)
cv2.imshow("cropped3", crop_img3)

crop_img3 = cv2.cvtColor(crop_img3, cv2.COLOR_BGR2GRAY)
crop_img3 = cv2.medianBlur(crop_img3, 5)
ret,crop_img3 = cv2.threshold(crop_img3, 200, 255, cv2.THRESH_BINARY)
cv2.imshow("cropped3_1", crop_img3)
crop_img3 = cv2.GaussianBlur(crop_img3, (3,3),4 )
data = pytesseract.image_to_string(crop_img3, lang='eng', config='--psm 6')
print(data)

cv2.imshow('thresh', crop_img3)
cv2.waitKey(0)
