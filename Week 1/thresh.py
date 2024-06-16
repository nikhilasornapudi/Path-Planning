import cv2.cv2 as cv2

img = cv2.imread('/Users/nikhilasornapudi/Downloads/Boston.jpeg')

cv2.imshow('Boston', img)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow('GRAYSCALE', gray)

# Simple Thresholding
threshold, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
cv2.imshow('Simple Thresholding', thresh)

threshold, thresh_inv = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
cv2.imshow('Simple Thresholding Inverse', thresh_inv)

# Adaptive Thresholding
adaptive_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 3)
cv2.imshow('Adaptive Thresholding', adaptive_thresh)

cv2.waitKey(0)
