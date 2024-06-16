import cv2 as cv2
import numpy as np

blank = np.zeros((500, 500, 3), dtype='uint8')
cv2.imshow('Blank', blank)

# 1. Paint the image a certain colour
# blank[200:300, 300:400] = 0, 255, 0
# cv.imshow('Green', blank)

# 2. Draw a rectangle
# cv.rectangle(blank, (0, 0), (250, 250), (0, 255, 0), thickness=2)
# cv.rectangle(blank, (0, 0), (250, 250), (0, 255, 0), thickness=cv.FILLED)
# cv2.rectangle(blank, (0, 0), (blank.shape[1] // 2, blank.shape[0] // 2), (0, 255, 0), thickness=-1)
# cv2.imshow('Rectangle', blank)
#
# 3. Draw a circle
# cv2.circle(blank, (blank.shape[1] // 2, blank.shape[0] // 2), 40, (255, 0, 0), thickness=3)
# cv2.imshow('Circle', blank)
#
# 4. Draw a Line
# cv2.line(blank, (0, 0), (blank.shape[1]//2, blank.shape[0]//2), (0, 0, 255), thickness=4)
# cv2.imshow('Line', blank)

# 5. Write text
cv2.putText(blank, 'Hello and Everyone!!', (0, 255), cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 2)
cv2.imshow('Line', blank)

# We can either pass in a standalone image or create a blank image to draw
# img = cv.imread('/Users/nikhilasornapudi/Downloads/Hiccup.jpg')
# cv.imshow('Hiccup', img)

cv2.waitKey(0)
