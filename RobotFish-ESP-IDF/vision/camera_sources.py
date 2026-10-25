"""Find usable camera indices using the same backend as the observer."""
import json

import cv2


def scan():
    cameras = []
    for index in range(10):
        capture = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        try:
            if not capture.isOpened():
                continue
            ok, frame = capture.read()
            if ok and frame is not None and frame.size:
                height, width = frame.shape[:2]
                cameras.append(dict(index=index, width=width, height=height))
        finally:
            capture.release()
    return {'cameras': cameras}


if __name__ == '__main__':
    print(json.dumps(scan()))
