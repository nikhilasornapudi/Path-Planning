import cv2 as cv

img = cv.imread('/Users/nikhilasornapudi/Downloads/Hiccup.jpg')

cv.imshow('Hiccup', img)


def rescaleFrame(frame, scale=0.75):
    # Images, Videos and Live videos
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)
    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)


def changeRes(width, height):
    # Live video
    capture.set(3, width)
    capture.set(4, height)


# reading videos
capture = cv.VideoCapture('/Users/nikhilasornapudi/Desktop/Macbook AIr.mp4')

while True:
    isTrue, frame = capture.read()

    frame_resized = rescaleFrame(frame, scale=0.2)

    cv.imshow('Video', frame)
    cv.imshow('Video Resized', frame_resized)

    if cv.waitKey(20) & 0xFF == ord('d'):
        break

capture.release()
cv.destroyAllWindows()
