import cv2 as cv2

img = cv2.imread('/Users/nikhilasornapudi/Downloads/image.jpg')
cv2.imshow('blah', img)

# Converting to greyscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow('GreyScale', gray)
#
# # Blur
blur = cv2.GaussianBlur(img, (7, 7), cv2.BORDER_DEFAULT)
cv2.imshow('Blurred Image', blur)
#
# # Edge Cascades
canny = cv2.Canny(blur, 125, 175)
cv2.imshow('Canny', canny)   # We can minimize the edges by passing the blurred image
#
# # Dilating the image
dilated = cv2.dilate(canny, (7, 7), iterations=3)
cv2.imshow('Dilated', dilated)
#
# # Eroding the image --> changing back to the structured image
eroded = cv2.erode(dilated, (7, 7), iterations=3)
cv2.imshow('Eroded', eroded)

# Resize an image
#resized = cv2.resize(img, (500, 500))
#cv2.imshow('Resized', resized)
# if you're trying to shrink the image than the original image u will be using interpolations=INTER_AREA
# whereas if you're trying to enlarge the image than the original image u will be using INTER_LINEAR/ INTER_CUBIC
# INTER_CUBIC is slowest among them all, yet it provides a high resolution image among them all

# Cropping an image
#cropped = img[50:200, 200:400]
#cv2.imshow('Cropped', cropped)

cv2.waitKey(0)
