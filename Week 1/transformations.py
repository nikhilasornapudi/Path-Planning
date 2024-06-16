
import cv2.cv2 as cv2
import numpy as np

img = cv2.imread('/Users/nikhilasornapudi/Downloads/Boston.jpeg')

cv2.imshow('Boston', img)


def translate(img, x, y):
    transMat = np.float32([[1, 0, x], [0, 1, y]])
    dimensions = (img.shape[1], img.shape[0])
    return cv2.warpAffine(img, transMat, dimensions)


# -x --> Left
# -y --> Up
# x ---> Right
# y ---> Down


translated = translate(img, 100, 100)
cv2.imshow('Translate', translated)


# Rotation of an image

def rotate(img, angle, rotPoint=None):
    (height, width) = img.shape[:2]

    if rotPoint is None:
        rotPoint = (width // 2, height // 2)

    rotMat = cv2.getRotationMatrix2D(rotPoint, angle, 1.0)
    dimensions = (width, height)

    return cv2.warpAffine(img, rotMat, dimensions)


rotated = rotate(img, 45)
cv2.imshow('Rotated', rotated)


# Resizing an image

resized = cv2.resize(img, (500, 500), interpolation=cv2.INTER_LINEAR)
cv2.imshow('Resized', resized)

# Flipping an image

flipped = cv2.flip(img, -1)
# 0 ---> flipping along the x-axis vertically
# 1 ---> flipping along the y-axis horizontally
# -1 --> flipping in both vertically and horizontally
cv2.imshow('Flipped', flipped)

# Cropping

cropped = img[200:400, 300:400]
cv2.imshow('Cropped', cropped)

cv2.waitKey(0)
