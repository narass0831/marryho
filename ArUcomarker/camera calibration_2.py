# calibrate_camera.py
import cv2
import numpy as np
import glob
import pickle

# 체스보드 내부 코너 수
chess_size = (9, 6)
square_size = 0.025  # 25mm → 단위: m

# 월드 좌표계 생성
objp = np.zeros((chess_size[0] * chess_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chess_size[0], 0:chess_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

# 촬영한 이미지 불러오기
images = glob.glob('./calib_imgs/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chess_size, None)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # 시각화 (옵션)
        cv2.drawChessboardCorners(img, chess_size, corners, ret)
        cv2.imshow('Chessboard', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# 카메라 보정
ret, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# 결과 저장
calib_data = {'camera_matrix': camera_matrix, 'dist_coeffs': dist_coeffs}
with open('camera_calibration.pkl', 'wb') as f:
    pickle.dump(calib_data, f)

print("카메라 보정 완료: camera_calibration.pkl 저장됨")

