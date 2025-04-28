import spidev
import time
from picamera2 import Picamera2
import torch
import cv2
import numpy as np
from gpiozero import DistanceSensor

# YOLOv5(nano) 모델 불러오기
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
model.classes = [0]  # 사람만 인식

# PiCamera2 설정
picam2 = Picamera2()
config = picam2.create_preview_configuration({"format": "RGB888", "size": (1200, 900)})
picam2.configure(config)

# 초음파 센서 설정
sensor = DistanceSensor(echo=20, trigger=21)

# 추종 활성화 상태 항상 True
tracking_enabled = True

# 카메라 시작
picam2.start()

# SPI 설정
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 50000
spi.mode = 0

first_person_id = None
first_target_x = None
first_target_y = None

def spi_send(command):
    spi.xfer2([ord(command)])
    print(f"SPI 전송: {command}")

try:
    while True:
        distance = sensor.distance * 100
        print(f"거리: {distance:.2f} cm")

        frame = picam2.capture_array()

        if not tracking_enabled:
            spi_send("S")
            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        results = model(frame)
        boxes = results.xyxy[0].cpu().numpy()

        if first_person_id is None:
            for idx, box in enumerate(boxes):
                x1, y1, x2, y2, confidence, cls = box
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                if confidence > 0.5:
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    first_person_id = idx
                    first_target_x = center_x
                    first_target_y = center_y
                    print(f"첫 번째 인식한 사람 ID: {first_person_id}")

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, "Target", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.circle(frame, (first_target_x, first_target_y), 5, (0, 0, 255), -1)
                    break

        if first_person_id is not None:
            if first_person_id < len(boxes):
                box = boxes[first_person_id]
                x1, y1, x2, y2, confidence, cls = box
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                first_target_x = (x1 + x2) // 2
                first_target_y = (y1 + y2) // 2

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, "Target", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.circle(frame, (first_target_x, first_target_y), 5, (0, 0, 255), -1)

                if distance <= 20:
                    spi_send("S")
                elif first_target_x < 200:
                    spi_send("L")
                elif first_target_x > 1000:
                    spi_send("R")
                else:
                    spi_send("F")
            else:
                spi_send("S")

        cv2.imshow("Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("종료")

finally:
    spi.close()
    picam2.stop()
    cv2.destroyAllWindows()