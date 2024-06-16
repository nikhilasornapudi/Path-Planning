
import cv2 as cv2
import numpy as np


img = cv2.imread('/Users/nikhilasornapudi/Downloads/image.jpg')
cv2.imshow('Boston', img)

blank = np.zeros(img.shape, dtype='uint8')
cv2.imshow('Blank', blank)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow('GreyScale', gray)

# blur = cv2.GaussianBlur(img, (5, 5), cv2.BORDER_DEFAULT)
# cv2.imshow('Blurred Image', blur)

canny = cv2.Canny(img, 125, 175)
cv2.imshow('Canny', canny)

# ret, thresh = cv2.threshold(gray, 125, 255, cv2.THRESH_BINARY)
# cv2.imshow('Thresh', thresh)

contours, hierarchies = cv2.findContours(canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
print(f'{len(contours)} contours are found!')

cv2.drawContours(blank, contours, -1, (0, 0, 255), 2)
cv2.imshow('Contours drawn', blank)


cv2.waitKey(0)
