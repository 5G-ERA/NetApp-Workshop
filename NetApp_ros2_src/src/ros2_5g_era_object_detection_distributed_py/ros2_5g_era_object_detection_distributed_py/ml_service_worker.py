# Celery worker

import logging
import os
import cv2
from celery import Celery

from typing import Dict  # for Python 3.9, just 'dict' will be fine

import numpy as np
from ros2_5g_era_helpers import get_path_to_assets


broker_url = os.environ.get("CELERY_BROKER_URL", "amqp://guest:guest@127.0.0.1:5672")
redis_url = os.environ.get("CELERY_RESULT_BACKEND", "redis://127.0.0.1/")
app = Celery('tasks', broker=broker_url, backend=redis_url, )
app.conf.task_serializer = 'pickle'
app.conf.result_serializer = 'pickle'
app.conf.accept_content = ['pickle']

path_to_assets = get_path_to_assets()
detection_cascade = cv2.CascadeClassifier(os.path.join(path_to_assets, 'haarcascade_frontalface_default.xml'))


@app.task
def detect_faces(data):
    metadata, image = data
            
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # gray = cv2.resize(gray,(640,360))
    # Detect the faces
    faces = detection_cascade.detectMultiScale(gray, 1.35, 4)

    detections = []
    
    for (x,y,w,h) in faces:

        bbox = [x,y,w,h]

        # Generate random class
        cls = 1 # np.random.randint(0, 80)

        # Generate random detection score
        score = np.random.random()
        det = (bbox, cls, score)

        # Add to other detections for processed frame
        detections.append(det)
    
    #task_id = metadata["node_name"] # TODO: remove

    results = (metadata, detections)
    return results

