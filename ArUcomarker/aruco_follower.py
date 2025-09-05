import spidev
import time
import cv2
import numpy as np
import pickle
from picamera2 import Picamera2
# from gpiozero import DistanceSensor  # 초음파 센서 (주석 유지)

# 1. 카메라 캘리브레이션 데이터 불러오기
with open('camera_calibration.pkl', 'rb') as f:
    calib = pickle.load(f)
camera_matrix = calib['camera_matrix']
dist_coeffs = calib['dist_coeffs']

# 2. SPI 설정
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 50000
spi.mode = 0

def spi_send(command):
    """SPI 명령 전송"""
    spi.xfer2([ord(command)])
    print(f"▶ SPI 전송: {command}")

# 3. ArUco 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
marker_length = 0.05  # 단위: m (5cm)

# 4. PiCamera2 설정
picam2 = Picamera2()
config = picam2.create_preview_configuration({"format": "RGB888", "size": (1080, 800)})
picam2.configure(config)
picam2.start()
time.sleep(2)

# 5. 초음파 센서 (필요시 활성화)
# sensor = DistanceSensor(echo=20, trigger=21)

print("▶ ArUco 마커 추종 시작")

try:
    while True:
        # 6. 초음파 거리 측정 (cm 단위)
        # distance = sensor.distance * 100
        # print(f"거리: {distance:.2f} cm")

        # 7. 카메라 프레임 획득 및 왜곡 보정
        frame = picam2.capture_array()
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)

        # 8. 마커 검출
        corners, ids, _ = detector.detectMarkers(undistorted)

        if ids is not None:
            # 9. Pose 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_length, camera_matrix, dist_coeffs
            )

            for i in range(len(ids)):
                center_x = int(np.mean(corners[i][0][:, 0]))
                center_y = int(np.mean(corners[i][0][:, 1]))
                x, y, z = tvecs[i][0]

                # 10. 이동 판단
                # if distance <= 40:  # 초음파 거리 사용 시
                if z < 0.4:  # ArUco 거리 기준
                    spi_send("S")
                elif center_x < 350:
                    spi_send("L")
                elif center_x > 730:
                    spi_send("R")
                else:
                    spi_send("F")

                # 11. 시각화
                cv2.aruco.drawDetectedMarkers(undistorted, corners, ids)
                cv2.drawFrameAxes(undistorted, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.03)
                cv2.circle(undistorted, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(undistorted, f"ID: {ids[i][0]}", (center_x - 30, center_y - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(undistorted, f"Z: {z:.2f}m", (center_x - 30, center_y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                break  # 첫 번째 마커만 추종

        else:
            # 마커가 없을 경우 정지
            spi_send("S")

        # 12. 화면 출력
        cv2.imshow("ArUco Tracker", undistorted)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("사용자 중단")

finally:
    spi.close()
    picam2.stop()
    cv2.destroyAllWindows()
    print("시스템 종료 완료")
