import cv2
import cvzone
import time
import numpy as np
from pyzbar.pyzbar import decode

# Thresholds
thres = 0.55
nmsThres = 0.2

# Load class names
classNames = []
classFile = '/home/drone/Desktop/dronekit-python/examples/control_drone_web/coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().strip().split('\n')

# Load model
configPath = '/home/drone/Desktop/dronekit-python/examples/control_drone_web/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = "/home/drone/Desktop/dronekit-python/examples/control_drone_web/frozen_inference_graph.pb"
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(1270, 720)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# Initialize webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

def object_detect():
    while True:
        success, img = cap.read()
        img = cv2.flip(img, 1)
        classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nmsThres)
        object_count = 0

        if len(classIds) != 0:
            for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
                if classNames[classId - 1].lower() == 'person':
                    cvzone.cornerRect(
                        img, box,
                        l=20, t=3, rt=1,
                        colorR=(208, 224, 64),
                        colorC=(255, 0, 0)
                    )
                    cv2.putText(img, f'{classNames[classId - 1].upper()}',
                                (box[0] + 10, box[1] + 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 2)
                    object_count += 1

        cv2.putText(img, f'Count: {object_count}', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Detection", img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyWindow("Detection")
            qr_detect()
        if key == ord('e'):
            break

def qr_detect():
    while True:
        success, img = cap.read()
        img = cv2.flip(img, 1)
        for barcode in decode(img):
            myData = barcode.data.decode('utf-8')
            print(myData)
            pts = np.array([barcode.polygon], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(img, [pts], True, (208, 224, 64), 3)
            pts2 = barcode.rect
            cv2.putText(img, myData, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

            x, y, w, h = pts2
            qr_crop = img[y:y + h, x:x + w]
            if qr_crop.size != 0:
                qr_zoom = cv2.resize(qr_crop, (200, 200))  # Resize for better visibility
                cv2.imshow('QR Code Zoom', qr_zoom)

        cv2.imshow("QR", img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('1'):
            cv2.destroyWindow("QR")
            try:
                cv2.destroyWindow("QR Code Zoom")
            except cv2.error:
                pass
            object_detect()
        if key == ord('e'):
            break

def main():
    object_detect()

    # Release the webcam and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
