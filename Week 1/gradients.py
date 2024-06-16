import cv2.cv2 as cv2
import numpy as np

img = cv2.imread('/Users/nikhilasornapudi/Downloads/6127292.jpg')
cv2.imshow('Hiccup', img)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow('GRAYSCALE', gray)

# An image gradient is defined as a directional change in image intensity.
# We use gradients for detecting edges in images, which allows us to find contours and outlines of objects in images

# Laplacian Gradient Magnitude Representation
# basically returns a pencil shading version of edges in the image

lap = cv2.Laplacian(gray, cv2.CV_64F)
lap = np.uint8(np.absolute(lap))
cv2.imshow('Laplacian', lap)

# Sobel Gradient Magnitude Representation
sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1)
combined_sobel = cv2.bitwise_or(sobel_x, sobel_y)

cv2.imshow('Sobel X axis', sobel_x)
cv2.imshow('Sobel Y axis', sobel_y)
cv2.imshow('Combined Gradient', combined_sobel)

canny = cv2.Canny(gray, 150, 175)
cv2.imshow('Canny', canny)

cv2.waitKey(0)

