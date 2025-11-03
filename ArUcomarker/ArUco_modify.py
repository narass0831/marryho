import spidev
import time
import cv2
import numpy as np
import pickle
from picamera2 import Picamera2

# ================== 하이퍼파라미터(조절) ==================
FULLSCAN_INTERVAL = 10     # N프레임마다 풀프레임 재검출
ROI_MARGIN_PX     = 80     # ROI 확장 마진 (px)
LOST_TOLERANCE    = 5      # ROI에서 연속 미검출 시 풀스캔 전환
TARGET_PICK       = "largest"  # "largest"(픽셀면적 최대) or "closest"(tz 최소)

# ========== 1) 카메라 캘리브레이션 데이터 ==========
with open('camera_calibration.pkl', 'rb') as f:
    calib = pickle.load(f)
camera_matrix = calib['camera_matrix']
dist_coeffs = calib['dist_coeffs']

# ========== 2) SPI 설정 ==========
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 50000
spi.mode = 0

def spi_send(command: str):
    spi.xfer2([ord(command)])
    print(f"▶ SPI 전송: {command}")

# ========== 3) ArUco 설정 ==========
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters()
# 속도형(정확도보다 FPS) 튜닝 예시
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
marker_length = 0.05  # [m] (5 cm)

# ========== 4) PiCamera2 설정 ==========
picam2 = Picamera2()
config = picam2.create_preview_configuration({"format": "RGB888", "size": (1080, 800)})
picam2.configure(config)
picam2.start()
time.sleep(2)

print("▶ ArUco 마커 추종 시작")

# ========== 5) FPS 측정 변수 ==========
frame_count = 0
window_start = time.perf_counter()
ema_fps = None
ema_alpha = 0.2
prev_frame_time = time.perf_counter()

# ========== 6) ROI 상태 ==========
roi_rect = None        # (x0, y0, x1, y1) or None
miss_in_roi = 0        # ROI 내 연속 미검출 카운트
frame_idx = 0

def clamp(a, lo, hi): return max(lo, min(a, hi))

def corners_to_rect(corners):
    """corners: (1,4,2) 또는 (4,2). 픽셀 AABB 반환"""
    pts = corners.reshape(-1, 2)
    x0, y0 = np.min(pts, axis=0)
    x1, y1 = np.max(pts, axis=0)
    return int(x0), int(y0), int(x1), int(y1)

def expand_rect(rect, margin, W, H):
    x0, y0, x1, y1 = rect
    x0 = clamp(x0 - margin, 0, W-1)
    y0 = clamp(y0 - margin, 0, H-1)
    x1 = clamp(x1 + margin, 0, W-1)
    y1 = clamp(y1 + margin, 0, H-1)
    return (x0, y0, x1, y1)

def offset_corners(corners, ox, oy):
    """ROI 좌표계 코너를 원본 좌표계로 변환"""
    c = corners.copy()
    c[..., 0] += ox
    c[..., 1] += oy
    return c

def pick_target(corners_list, ids, tvecs=None):
    """가장 큰 면적(또는 가장 가까운) 마커 선택, 인덱스 반환"""
    if ids is None or len(ids) == 0:
        return None
    if TARGET_PICK == "closest" and tvecs is not None:
        zs = [tvecs[i][0,2] for i in range(len(ids))]
        return int(np.argmin(zs))
    # default: largest area in image space
    areas = []
    for i in range(len(ids)):
        x0,y0,x1,y1 = corners_to_rect(corners_list[i])
        areas.append((x1-x0)*(y1-y0))
    return int(np.argmax(areas))

try:
    while True:
        frame_idx += 1

        # ---- 프레임 캡처 & 왜곡 보정 ----
        frame = picam2.capture_array()
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
        H, W = undistorted.shape[:2]

        # ---- ROI 사용할지 결정 ----
        use_roi = roi_rect is not None \
                  and (frame_idx % FULLSCAN_INTERVAL != 0) \
                  and (miss_in_roi < LOST_TOLERANCE)

        if use_roi:
            x0, y0, x1, y1 = roi_rect
            roi = undistorted[y0:y1, x0:x1]
            corners, ids, _ = detector.detectMarkers(roi)
            if ids is not None and len(ids) > 0:
                # ROI 좌표계 → 원본 좌표계로 코너 오프셋
                corners = [offset_corners(c, x0, y0) for c in corners]
            else:
                # ROI에서 놓쳤으면 카운트 올리고, 이번 프레임은 í프레임로 대체
                miss_in_roi += 1
                corners, ids, _ = detector.detectMarkers(undistorted)
        else:
            corners, ids, _ = detector.detectMarkers(undistorted)

        if ids is not None and len(ids) > 0:
            miss_in_roi = 0  # 보이면 초기화

            # ---- 타깃 선택 ----
            idx = pick_target(corners, ids)
            if idx is None:
                spi_send("S")
            else:
                # ---- 포즈 추정(원본 좌표계 코너 사용!) ----
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners[idx]], marker_length, camera_matrix, dist_coeffs
                )
                rvec = rvecs[0]; tvec = tvecs[0]
                x, y, z = tvec[0]

                # ---- 중심 좌표 ----
                cx = int(np.mean(corners[idx][0][:,0]))
                cy = int(np.mean(corners[idx][0][:,1]))

                # ---- 제어 판단 ----
                if z < 0.4:
                    spi_send("S")
                elif cx < 350:
                    spi_send("L")
                elif cx > 730:
                    spi_send("R")
                else:
                    spi_send("F")

                # ---- 시각화 ----
                cv2.aruco.drawDetectedMarkers(undistorted, corners, ids)
                cv2.drawFrameAxes(undistorted, camera_matrix, dist_coeffs, rvec, tvec, 0.03)
                cv2.circle(undistorted, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(undistorted, f"ID: {ids[idx][0]}", (cx - 30, cy - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(undistorted, f"Z: {z:.2f}m", (cx - 30, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # ---- 다음 프레임 ROI 갱신 ----
                # (선택된 마커의 AABB + margin → 프레임 내로 클램프)
                rect = corners_to_rect(corners[idx])
                roi_rect = expand_rect(rect, ROI_MARGIN_PX, W, H)

        else:
            # 전혀 보이지 않으면 정지 & ROI 약화
            spi_send("S")
            miss_in_roi += 1
            if miss_in_roi >= LOST_TOLERANCE:
                roi_rect = None  # ROI 포기 (다음 프레임은 풀스캔 유지)

        # ---- FPS 측정 ----
        now = time.perf_counter()
        dt = now - prev_frame_time
        prev_frame_time = now
        inst_fps = (1.0 / dt) if dt > 0 else 0.0
        ema_fps = inst_fps if ema_fps is None else (ema_alpha * inst_fps + (1 - ema_alpha) * ema_fps)

        frame_count += 1
        elapsed = now - window_start
        if elapsed >= 2.0:
            avg_fps = frame_count / elapsed
            print(f"▶ FPS (2s avg): {avg_fps:.2f} / EMA: {ema_fps:.2f} / mode: {'ROI' if use_roi else 'FULL'}")
            frame_count = 0
            window_start = now

        # ---- 디버그용 ROI 박스 그리기 ----
        if roi_rect is not None:
            x0,y0,x1,y1 = roi_rect
            cv2.rectangle(undistorted, (x0,y0), (x1,y1), (0,255,0), 2)

        cv2.putText(undistorted, f"FPS: {ema_fps:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (50, 255, 50), 2)
        cv2.imshow("ArUco Tracker", undistorted)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("▶ 사용자 중단")

finally:
    spi.close()
    picam2.stop()
    cv2.destroyAllWindows()
    print("▶ 시스템 종료 완료")