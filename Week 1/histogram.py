import cv2.cv2 as cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('/Users/nikhilasornapudi/Downloads/Boston.jpeg')

blank = np.zeros(img.shape[:2], dtype='uint8')
cv2.imshow('Boston', img)

# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# cv2.imshow('GreyScale', gray)

mask = cv2.circle(blank, (img.shape[1]//2, img.shape[0]//2), 100, 255, -1)

masked = cv2.bitwise_and(img, img, mask=mask)
cv2.imshow('Mask', masked)

# Grayscale histogram

# gray_hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
#
# plt.figure()
# plt.title('Grayscale Histogram')
# plt.xlabel('Bins')
# plt.ylabel('# of pixels')
# plt.plot(gray_hist)
# plt.xlim([0, 256])
# plt.show()

# Color Histogram

plt.figure()
plt.title('Colour Histogram')
plt.xlabel('Bins')
plt.ylabel('# of pixels')

colours = ('b', 'g', 'r')
for i, col in enumerate(colours):
    hist = cv2.calcHist([img], [i], mask, [256], [0, 256])
    plt.plot(hist, color=col)
    plt.xlim([0, 256])

plt.show()

cv2.waitKey(0)
