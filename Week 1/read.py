import cv2 as cv

# img = cv.imread('/Users/nikhilasornapudi/Downloads/Hiccup.jpg')

# cv.imshow('Hiccup', img)

capture = cv.VideoCapture('/Users/nikhilasornapudi/Desktop/Macbook AIr.mp4')

while True:
    isTrue, frame = capture.read()
    cv.imshow('Video', frame)

    if cv.waitKey(20) & 0xFF == ord('d'):
        break 

capture.release()
cv.destroyAllWindows()




