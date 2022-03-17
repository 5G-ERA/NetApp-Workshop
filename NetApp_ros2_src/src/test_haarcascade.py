
import cv2

# Load the cascade
# face_cascade = cv2.CascadeClassifier('assets/haarcascade_frontalface_extended.xml')
face_cascade = cv2.CascadeClassifier('assets/haarcascade_frontalface_default.xml')

# To capture video from webcam. 
# cap = cv2.VideoCapture(0)
# To use a video file as input 
cap = cv2.VideoCapture('assets/test_video.mp4')

while True:
    # Read the frame
    _, img = cap.read()

    img = cv2.resize(img, (1280,720))
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # gray = cv2.resize(gray,(640,360))
    # Detect the faces
    faces = face_cascade.detectMultiScale(gray, 1.35, 4)
    # Draw the rectangle around each face
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
    # Display
    cv2.imshow('img', img)
    print(type(img))
    # Stop if escape key is pressed
    k = cv2.waitKey(30) & 0xff
    if k==27:
        break
# Release the VideoCapture object
cap.release()