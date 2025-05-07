from picamera2 import Picamera2 # Raspberry Pi 카메라 모듈을 제어하기 위한 라이브러리
import torch # YOLOv5 모델을 불러오기 위한 PyTorch 라이브러리
import cv2 # OpenCV 라이브러리 (영상 처리용)
import numpy as np # 행렬 및 이미지 데이터 처리를 위한 NumPy 라이브러리
# YOLOv5 모델 불러오기 (COCO 데이터셋 - 사람 인식 전용)
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)	
model.classes = [0] # COCO 클래스 중 '사람'만 인식 (class 0 = person)
# PiCamera2 설정
picam2 = Picamera2() # Picamera2 객체 생성
config = picam2.create_preview_configuration({"format": "RGB888", "size": (640, 480)}) # 해상도 및 포맷 설정
picam2.configure(config) # 설정 적용
# 카메라 시작
picam2.start()
while True:
 # 프레임 가져오기 (NumPy 배열 형식으로 획득)
 frame = picam2.capture_array()
 # YOLOv5 모델을 사용하여 객체 탐지 수행
 results = model(frame)
 # 결과에서 사람만 추출 (바운딩 박스 좌표 및 신뢰도 값 획득)
 boxes = results.xyxy[0].cpu().numpy()
 # 현재 프레임에서 인식된 사람 좌표 목록
 current_people = []
 # 사람 바운딩 박스 그리기
 for box in boxes:
 x1, y1, x2, y2, confidence, cls = box # 바운딩 박스 좌표 및 신뢰도 값 획득
 x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # 정수형 변환
 # 신뢰도 (Confidence)가 0.5 이상인 경우 바운딩 박스 표시
 if confidence >0.5:
 # 현재 사람의 중심 좌표 계산
 center_x = (x1 + x2) // 2
 center_y = (y1 + y2) // 2
 current_people.append((center_x, center_y, x1, y1, x2, y2))
 # 바운딩 박스 및 텍스트 표시 (초록색)
 cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2) # 초록색 박스 그리기
 cv2.putText(frame, "Person", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2) # 텍스트 표시
 # 원본 영상 출력 (바운딩 박스 포함)
 cv2.imshow("Camera", frame)
 # 'q' 키를 누르면 종료
 if cv2.waitKey(1) &0xFF == ord('q'):
 break
# 카메라 종료
picam2.stop()
cv2.destroyAllWindows()