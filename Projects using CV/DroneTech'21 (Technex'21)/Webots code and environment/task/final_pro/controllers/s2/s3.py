import cv2
import numpy as np
import math
import pytesseract
pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'
frame = cv2.imread('first in bgr.jpeg')
cv2.imshow('frame', frame)
frame=cv2.resize(frame,(400,400))
cv2.imshow('frame',frame)
# hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# lower = np.array([20, 52, 72], np.uint8)
# upper = np.array([100, 255, 255], np.uint8)
# median = cv2.medianBlur(hsv1, 7)
# full_mask = cv2.inRange(median, lower, upper)
# corners = cv2.goodFeaturesToTrack(full_mask, 4, 0.01, 300)
# for corner in corners:
#     x, y = corner.ravel()
#     cv2.circle(full_mask, (x,y), 8, 100,-1)
# cv2.imshow('mask',full_mask)
#frame=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
#cv2.imshow('rgb',frame2)

#green masking+edge detection+centre finding
hsvg = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
green_lower = np.array([25, 52, 72], np.uint8)
green_upper = np.array([102, 255, 255], np.uint8)
median_g = cv2.medianBlur(hsvg, 7)
green_mask = cv2.inRange(median_g, green_lower, green_upper)
green_mask = cv2.erode(green_mask, (5,5), iterations=4)
green_mask = cv2.erode(green_mask, (5,5), iterations=4)
cv2.imshow('eroded green1', green_mask)
corners = cv2.goodFeaturesToTrack(green_mask, 4, 0.001, 100)
corners = np.int0(corners)
edgesg = np.zeros((4,2))
i = 0
gcx = 0
gcy = 0
for corner in corners:
    x, y = corner.ravel()
    edgesg[i,:]=[x,y]
    i+=1
    gcx=x+gcx
    gcy=y+gcy
green_center = (int(gcx / 4), int(gcy/4))
print(edgesg)
edgesg = edgesg[edgesg[:,0].argsort()]



hsvy = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
yellow_lower = np.array([20, 100, 100], np.uint8)
yellow_upper = np.array([30, 255, 255], np.uint8)
yellow_median = cv2.medianBlur(hsvy, 7)
yellow_mask = cv2.inRange(yellow_median, yellow_lower, yellow_upper)
corners = cv2.goodFeaturesToTrack(yellow_mask, 4, 0.01, 60)
i=0
ycx=0
ycy=0
edgesy = np.zeros((4,2))
for corner in corners:
    x, y = corner.ravel()
    edgesy[i, :] = [x, y]
    ycx+=x
    ycy+=y
    i+=1
    cv2.circle(yellow_mask, (x,y), 8, 100,-1)
yellow_center = (int(ycx / 4), int(ycy/ 4))
center=(int((green_center[0]+yellow_center[0])/2), int((green_center[1]+yellow_center[1])/2))
cv2.imshow('yellow mask', yellow_mask)
edgesy = edgesy[edgesy[:,0].argsort()]
#Matrix = cv2.getRotationMatrix2D(center, theta, 1.0)
#frame = cv2.warpAffine(frame, Matrix, (400, 400))
#blank=np.zeros((500,500,3), np.uint8)
theta=math.degrees(math.atan((green_center[1]-yellow_center[1])/((green_center[0]-yellow_center[0]))))
#theta=math.degrees(math.atan((edgesy[0,1]-edgesy[2,1])/(edgesy[2,0]-edgesy[0,0])))
print('theta1',theta)
#print('theta',theta)
Matrix = cv2.getRotationMatrix2D(center, theta, 1.0)
frame = cv2.warpAffine(frame, Matrix, (400, 400))
cv2.imshow('frame rotated', frame)
#cv2.imshow('blank',blank)
#cv2.circle(frame, center, 8, 100, -1)
#cv2.circle(frame, (green_center[0],green_center[1]), 16, (0,0,100), -1)
print('original centre', center)
print('green centre', green_center)
cv2.imshow('frame rotated',frame)
print('original centres(y/g)')
print(yellow_center, green_center)
# theta=math.degrees(math.atan((edges[0,1]-edges[1,1])/(edges[0,0]-edges[1,0])))
# Matrix = cv2.getRotationMatrix2D(center, theta, 1.0)
# frame = cv2.warpAffine(frame, Matrix, (400, 400))
hsvg = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
green_lower = np.array([25, 52, 72], np.uint8)
green_upper = np.array([102, 255, 255], np.uint8)
median_g = cv2.medianBlur(hsvg, 5)
green_mask = cv2.inRange(median_g, green_lower, green_upper)
green_mask = cv2.erode(green_mask, (5,5), iterations=3)
green_mask = cv2.erode(green_mask, (3,3), iterations=2)
# cv2.imshow('eroded green', green_mask)
mask = np.zeros_like(frame) # Create mask where white is what we want, black otherwise
dc, contours = cv2.findContours(green_mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
c = max(dc, key=cv2.contourArea)
cv2.drawContours(mask, c, 255, -1) # Draw filled contour in mask
x,y,w,h = cv2.boundingRect(c)
crop_img = frame[y:y+h, x:x+w]
cv2.imshow('output', crop_img)
corners = cv2.goodFeaturesToTrack(green_mask, 4, 0.01, 130)
corners = np.int0(corners)
edgesg = np.zeros((4,2))
print(edgesg)
i = 0
gcx = 0
gcy = 0
for corner in corners:
    x, y = corner.ravel()
    edgesg[i,:]=[x,y]
    i+=1
    gcx=x+gcx
    gcy=y+gcy
    cv2.circle(green_mask, (x,y), 10, 255,-1)
green_center1 = (int(gcx / 4), int(gcy/4))
edgesg = edgesg[edgesg[:,0].argsort()]
print('rotated edgesg', edgesg)
#green_mask = cv2.erode(green_mask, (7,7), iterations=5)
cv2.imshow('eroded green', green_mask)

hsvy = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
yellow_lower = np.array([20, 100, 100], np.uint8)
yellow_upper = np.array([30, 255, 255], np.uint8)
yellow_median = cv2.medianBlur(hsvy, 7)
yellow_mask = cv2.inRange(yellow_median, yellow_lower, yellow_upper)
corners = cv2.goodFeaturesToTrack(yellow_mask, 4, 0.01, 100)
i=0
ycx=0
ycy=0
edgesy = np.zeros((4,2))
for corner in corners:
    x, y = corner.ravel()
    edgesy[i, :] = [x, y]
    ycx+=x
    ycy+=y
    i+=1

yellow_center1 = (int(ycx / 4), int(ycy/ 4))
print('rotated centres(y,g)')
print(yellow_center, green_center)
cv2.imshow('yellow rotated', yellow_mask)
center=(int((green_center[0]+green_center[1])/2), int((yellow_center[0]+yellow_center[1])/2))
print('new center', center)
if(yellow_center1[0]<green_center1[0]):
    #x = int(edgesg[0, 0])
    #y = int(edgesg[0, 1])
    # c = (int(edgesg[1, 0] - edgesg[0, 0]))*(int(edgesg[1, 0] - edgesg[0, 0]))
    # b = (int(edgesg[1, 1] - edgesg[1, 0]))*(int(edgesg[1, 1] - edgesg[1, 0]))
    #w = int(edgesg[3,0]-edgesg[0,0])-10
    # print('w=',w)
    # Create mask where white is what we want, black otherwise
    dc, contours = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(dc, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    mask = np.zeros((w-20,h-20),np.uint8)
    cv2.drawContours(mask, c, 255, -1)  # Draw filled contour in mask
    crop_img = frame[(y+20):(y + h-20), (x+20):(x + w-20)]
    cv2.imshow('output', mask)
    cv2.imshow('output1', crop_img)
    M = cv2.moments(c)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    cv2.circle(crop_img, (mask.shape[0],mask.shape[0]), 10, 255,-1)
    cv2.imshow('output1', crop_img)
    print(cX,cY)
    Matrix = cv2.getRotationMatrix2D((mask.shape[0]//2,mask.shape[0]//2), 180, 1.0)
    crop_img = cv2.warpAffine(crop_img, Matrix, ((mask.shape[0],mask.shape[0])))
    cv2.imshow('letter', crop_img)
    ret, crop_img = cv2.threshold(crop_img, 200, 220, cv2.THRESH_BINARY)
    edged = cv2.Canny(crop_img, 30, 200)
    cv2.imshow('crop_img', crop_img)
    # dc, contours,abc = cv2.findContours(edged,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # c = max(contours, key=cv2.contourArea)
    # blank=np.zeros((700,700,3),np.uint8)
    # cv2.drawContours(blank, c, -1, (255), -1)
    data = pytesseract.image_to_string(crop_img, lang='eng', config='--psm 6')
    for i in data:
        if(ord(i)>=65 and ord(i)<=90):
            print(i)
            break
    # # cv2.imshow('blank', blank)
    #print(data)

elif(yellow_center1[0]>green_center1[0]):
    dc, contours = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(dc, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    mask = np.zeros((w-20,h-20),np.uint8)
    cv2.drawContours(mask, c, 255, -1)  # Draw filled contour in mask
    crop_img = frame[(y+20):(y + h-20), (x+20):(x + w-20)]
    cv2.imshow('output', mask)
    cv2.imshow('output1', crop_img)
    M = cv2.moments(c)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    cv2.circle(crop_img, (mask.shape[0],mask.shape[0]), 10, 255,-1)
    cv2.imshow('output1', crop_img)
    print(cX,cY)
    cv2.imshow('letter', crop_img)
    ret, crop_img = cv2.threshold(crop_img, 200, 220, cv2.THRESH_BINARY)
    edged = cv2.Canny(crop_img, 30, 200)
    cv2.imshow('crop_img', crop_img)
    # dc, contours,abc = cv2.findContours(edged,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # c = max(contours, key=cv2.contourArea)
    # blank=np.zeros((700,700,3),np.uint8)
    # cv2.drawContours(blank, c, -1, (255), -1)
    data = pytesseract.image_to_string(crop_img, lang='eng', config='--psm 6')
    for i in data:
        if(ord(i)>=65 and ord(i)<=90):
            print(i)
            break
    # # cv2.imshow('blank', blank)
    #print(data)
cv2.waitKey(0)
