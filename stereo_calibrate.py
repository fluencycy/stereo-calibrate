#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import os
import yaml

# 参数：棋盘格内角点尺寸和方格大小
pattern_size = (9, 6)        # 棋盘格内角点数量 (列, 行)
square_size = 0.025          # 棋盘格方格边长 (米)，请根据实际棋盘调整

# 图像文件所在目录
image_dir = os.path.expanduser("~/stereo_calib_images")
left_images = sorted(glob.glob(os.path.join(image_dir, 'left_*.png')))
right_images = sorted(glob.glob(os.path.join(image_dir, 'right_*.png')))
if len(left_images) == 0 or len(right_images) == 0:
    print("未找到标定图像，请检查图像保存路径。")
    exit(1)
if len(left_images) != len(right_images):
    print("左右图像数量不一致，请确认图像对齐。")
    exit(1)

# 准备棋盘格在世界坐标系中的坐标
pattern_points = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size  # 按实际方格大小缩放

obj_points = []    # 3D点坐标列表（世界坐标系）
img_points_left = []   # 左图像角点像素坐标
img_points_right = []  # 右图像角点像素坐标

# 遍历所有图像对，查找棋盘格角点
for left_file, right_file in zip(left_images, right_images):
    img_left = cv2.imread(left_file)
    img_right = cv2.imread(right_file)
    gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
    ret_l, corners_l = cv2.findChessboardCorners(gray_left, pattern_size,
                        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    ret_r, corners_r = cv2.findChessboardCorners(gray_right, pattern_size,
                        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ret_l and ret_r:
        # 细化角点坐标
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        cv2.cornerSubPix(gray_left, corners_l, (11,11), (-1,-1), criteria)
        cv2.cornerSubPix(gray_right, corners_r, (11,11), (-1,-1), criteria)
        obj_points.append(pattern_points)
        img_points_left.append(corners_l)
        img_points_right.append(corners_r)
    else:
        print(f"棋盘角点未在图像对 {left_file} 和 {right_file} 上同时检测到，已跳过该对。")

# 校验至少有足够的图像对用于标定
num_pairs = len(obj_points)
if num_pairs < 3:
    print("标定图像对数量不足，至少需要3对以上有效棋盘图像。")
    exit(1)

# 标定单目标定参数（为了获得初始内参）
image_shape = gray_left.shape[::-1]  # (宽, 高)
ret_l, M1, d1, rvecs_l, tvecs_l = cv2.calibrateCamera(obj_points, img_points_left, image_shape, None, None)
ret_r, M2, d2, rvecs_r, tvecs_r = cv2.calibrateCamera(obj_points, img_points_right, image_shape, None, None)
print(f"单目标定完成：左相机误差={ret_l:.4f}像素，右相机误差={ret_r:.4f}像素")

# 立体标定（计算相机间旋转和平移）
flags = cv2.CALIB_FIX_INTRINSIC  # 固定单目标定的内参，求解外参
criteria_stereo = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
retStereo, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
    obj_points, img_points_left, img_points_right, M1, d1, M2, d2, image_shape, criteria=criteria_stereo, flags=flags)
print(f"立体标定完成：重投影误差={retStereo:.4f}像素")
# 打印基线距离（两相机光心间距）
baseline = np.linalg.norm(T)
print(f"相机基线距离: {baseline:.4f} 米")

# 计算立体校正参数（使左右视图共面、行对准）
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(M1, d1, M2, d2, image_shape, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
# 保存标定结果到 YAML 文件
calib_left = {
    "image_width": image_shape[0],
    "image_height": image_shape[1],
    "camera_name": "stereo_left",
    "camera_matrix": {"rows": 3, "cols": 3, "data": M1.flatten().tolist()},
    "distortion_model": "plumb_bob",
    "distortion_coefficients": {"rows": 1, "cols": len(d1.flatten()), "data": d1.flatten().tolist()},
    "rectification_matrix": {"rows": 3, "cols": 3, "data": R1.flatten().tolist()},
    "projection_matrix": {"rows": 3, "cols": 4, "data": P1.flatten().tolist()}
}
calib_right = {
    "image_width": image_shape[0],
    "image_height": image_shape[1],
    "camera_name": "stereo_right",
    "camera_matrix": {"rows": 3, "cols": 3, "data": M2.flatten().tolist()},
    "distortion_model": "plumb_bob",
    "distortion_coefficients": {"rows": 1, "cols": len(d2.flatten()), "data": d2.flatten().tolist()},
    "rectification_matrix": {"rows": 3, "cols": 3, "data": R2.flatten().tolist()},
    "projection_matrix": {"rows": 3, "cols": 4, "data": P2.flatten().tolist()}
}
# 将参数写入YAML文件
with open("left.yaml", "w") as f:
    yaml.safe_dump(calib_left, f, sort_keys=False)
with open("right.yaml", "w") as f:
    yaml.safe_dump(calib_right, f, sort_keys=False)
print("标定参数已保存至 left.yaml 和 right.yaml")
