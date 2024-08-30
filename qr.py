from picamera2 import Picamera2, Preview
import cv2
import cvzone
import numpy as np
from pyzbar.pyzbar import decode

class ObjectAndQrDetector:
    def __init__(self):
        # Initialize thresholds
        self.thres = 0.55
        self.nmsThres = 0.2

        # Load class names
        self.classNames = []
        classFile = '/home/drone/Desktop/dronekit-python/examples/control_drone_web/coco.names'
        with open(classFile, 'rt') as f:
            self.classNames = f.read().strip().split('\n')

        # Load model
        configPath = '/home/drone/Desktop/dronekit-python/examples/control_drone_web/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        weightsPath = "/home/drone/Desktop/dronekit-python/examples/control_drone_web/frozen_inference_graph.pb"
        self.net = cv2.dnn_DetectionModel(weightsPath, configPath)
        self.net.setInputSize(320, 320)
        self.net.setInputScale(1.0 / 127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)

        # Start with object detection mode
        self.mode = 'object'

    def object_detect(self, img):
        classIds, confs, bbox = self.net.detect(img, confThreshold=self.thres, nmsThreshold=self.nmsThres)
        object_count = 0

        if len(classIds) != 0:
            for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
                if self.classNames[classId - 1].lower() == 'person':
                    object_count += 1
                    cvzone.cornerRect(
                        img, box,
                        l=20, t=3, rt=1,
                        colorR=(208, 224, 64),
                        colorC=(255, 0, 0)
                    )
                    cv2.putText(img, f'{self.classNames[classId - 1].upper()}',
                                (box[0] + 10, box[1] + 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 2)

        return img, object_count

    # def object_detect(self, img):
    #     # Perform object detection
    #     classIds, confs, bbox = self.net.detect(img, confThreshold=self.thres, nmsThreshold=self.nmsThres)
    #     object_count = 0

    #     # If any objects are detected
    #     if len(classIds) != 0:
    #         # Iterate over the detected objects
    #         for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
    #             # Check if the detected object is a person
    #             if self.classNames[classId - 1].lower() == 'person':
    #                 object_count += 1
    #                 # Draw a rectangle around the detected person
    #                 x, y, w, h = box
    #                 cv2.rectangle(img, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=2)
    #                 # Optionally, add the confidence level
    #                 cv2.putText(img, f'Person: {conf:.2f}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    #     # Return the modified frame and the count of detected persons
    #     return img, object_count

    
    # def qr_detect(self, img):
    #     qr_codes = []
    #     for barcode in decode(img):
    #         myData = barcode.data.decode('utf-8')
    #         qr_codes.append(myData)
    #     return img, qr_codes

    def qr_detect(self, img):
        # List to store the decoded QR code data
        qr_codes = []

        # Decode the QR codes in the image
        for barcode in decode(img):
            # Decode the QR code data
            myData = barcode.data.decode('utf-8')
            qr_codes.append(myData)

            # Get the coordinates of the bounding box
            pts = barcode.polygon
            if len(pts) == 4:
                # Draw a rectangle around the QR code
                pts = [(pt.x, pt.y) for pt in pts]
                npts = cv2.convexHull(np.array(pts, dtype=np.float32))
                cv2.polylines(img, [np.int32(npts)], isClosed=True, color=(0, 255, 0), thickness=2)

            # Get the bounding box for putting the text
            rect = barcode.rect
            cv2.putText(img, myData, (rect.left, rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Return the modified frame and the list of QR code data
        return img, qr_codes
    
    def run(self):
        while True:
            img = self.picam2.capture_array()

            # Ensure the image has 3 channels (RGB)
            if img.shape[2] == 4:
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

            if self.mode == 'object':
                object_count = self.object_detect(img)
                print(f'Objects detected: {object_count}')
            elif self.mode == 'qr':
                qr_codes = self.qr_detect(img)
                if qr_codes:
                    print(f'QR codes detected: {qr_codes}')

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.mode = 'qr'
            elif key == ord('1'):
                self.mode = 'object'
            elif key == ord('e'):
                break

        # Release the camera
        self.picam2.close()

if __name__ == "__main__":
    detector = ObjectAndQrDetector()
    detector.run()
