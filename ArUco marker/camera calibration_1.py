import cv2
import time
import os
from picamera2 import Picamera2

# 저장 폴더 생성
save_path = "./calib_imgs/"
os.makedirs(save_path, exist_ok=True)

# PiCamera2 설정
picam2 = Picamera2()
config = picam2.create_preview_configuration({"format": "RGB888", "size": (640, 480)})
picam2.configure(config)
picam2.start()
time.sleep(2)  # 카메라 워밍업

print("▶ 체스보드 사진 20장 촬영 시작")
for i in range(20):
    frame = picam2.capture_array()  # 프레임 캡처

    # 이미지 저장
    filename = f"{save_path}/chess_{i}.jpg"
    cv2.imwrite(filename, frame)
    print(f"[{i+1}/20] 저장됨: {filename}")

    # 실시간 미리보기 창에 표시
    cv2.putText(frame, f"Saved: {i+1}/20", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.imshow("Camera Preview", frame)

    # 키보드 ESC 누르면 중단 가능
    if cv2.waitKey(1) & 0xFF == 27:  # ESC 키
        print("사용자 중단: ESC 입력")
        break

    time.sleep(1.5)

# 종료 처리
picam2.stop()
cv2.destroyAllWindows()
print("캘리브레이션용 이미지 촬영 완료!")