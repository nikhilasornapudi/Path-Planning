import os
import cv2.cv2 as cv2
import cv2.face
import numpy as np

people = ['Selena Gomez', 'Shawn Mendes', 'Charlie Puth', 'Taylor Swift', 'Ed Sheeran']
DIR = r'/Users/nikhilasornapudi/Pictures/Train'
haar_cascade = cv2.CascadeClassifier('haar_face.xml')

features = []
labels = []


def create_train():
    for person in people:
        path = os.path.join(DIR, person)
        label = people.index(person)

        for img in os.listdir(path):
            img_path = os.path.join(path, img)

            img_array = cv2.imread(img_path)
            if img_array is None:
                continue

            gray = cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY)

            faces_rect = haar_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=7)

            for (x, y, w, h) in faces_rect:
                faces_roi = gray[y:y + h, x:x + w]
                features.append(faces_roi)
                labels.append(label)


create_train()
print('Training Done ---------------------------------------')

features = np.array(features, dtype='object')
labels = np.array(labels)

face_recognizer = cv2.face.LBPHFaceRecognizer_create()

# Train the recognizer on the features list and the labels list

face_recognizer.train(features, labels)

face_recognizer.save('face_trained.yml')
np.save('features.npy', features)
np.save('labels.npy', labels)

# print(f'Number of faces found in the image = {len(features)}')
# print(f'Number of faces found in the image = {len(labels)}')
