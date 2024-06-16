import cv2.cv2 as cv2

haar_cascade = cv2.CascadeClassifier('haar_face.xml')

people = ['Selena Gomez', 'Shawn Mendes', 'Charlie Puth', 'Taylor Swift', 'Ed Sheeran']
# features = np.load('features.npy')
# labels = np.load('labels.npy')

face_recognizer = cv2.face.LBPHFaceRecognizer_create()
face_recognizer.read('face_trained.yml')

img = cv2.imread(r'/Users/nikhilasornapudi/Pictures/Valid/Selena Gomez/7.jpeg')

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow('Person', gray)

# Detect the faces in the image
faces_rect = haar_cascade.detectMultiScale(gray, 1.1, 4)

for(x, y, w, h) in faces_rect:
    faces_roi = gray[y:y+h, x:x+w]

    label, confidence = face_recognizer.predict(faces_roi)
    print(f'Label =  {people[label]} with a confidence of {confidence}')

    cv2.putText(img, str(people[label]), (20, 20), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 255, 0), thickness=2)
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), thickness=2)

cv2.imshow('Detected Face', img)

cv2.waitKey(0)
