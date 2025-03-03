#!/usr/bin/env python3
import rospy
import cv2
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer

# 参数配置
pattern_size = (9, 6)   # 棋盘格内角点尺寸 (列数, 行数)
output_dir = os.path.expanduser("~/stereo_calib_images")  # 保存图像的目录
left_image_topic = rospy.get_param('~left_image_topic', '/left/image_raw')
right_image_topic = rospy.get_param('~right_image_topic', '/right/image_raw')

# 创建保存目录
if not os.path.isdir(output_dir):
    os.makedirs(output_dir)

# 初始化ROS节点
rospy.init_node('stereo_image_capture', anonymous=True)
bridge = CvBridge()
count = 0
last_save_time = rospy.Time.now()

# 定义棋盘格查找参数
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK

def image_callback(left_msg, right_msg):
    global count, last_save_time
    # 将ROS图像消息转换为OpenCV图像
    try:
        left_img = bridge.imgmsg_to_cv2(left_msg, "bgr8")
        right_img = bridge.imgmsg_to_cv2(right_msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"Image conversion error: {e}")
        return

    # 转为灰度图用于角点检测
    gray_left = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

    ret_left, corners_left = cv2.findChessboardCorners(gray_left, pattern_size, find_flags)
    ret_right, corners_right = cv2.findChessboardCorners(gray_right, pattern_size, find_flags)
    if ret_left and ret_right:
        # 若两幅图像均找到角点，检查是否满足保存时间间隔
        now = rospy.Time.now()
        if now - last_save_time < rospy.Duration(1.0):  # 默认1秒间隔，避免重复保存相同姿态
            return
        last_save_time = now
        # 可选：细化角点坐标提高精度（此步对保存图像本身无影响，可在标定时细化）
        cv2.cornerSubPix(gray_left, corners_left, (11,11), (-1,-1), criteria)
        cv2.cornerSubPix(gray_right, corners_right, (11,11), (-1,-1), criteria)
        # 保存图像对
        left_path = os.path.join(output_dir, f"left_{count}.png")
        right_path = os.path.join(output_dir, f"right_{count}.png")
        cv2.imwrite(left_path, left_img)
        cv2.imwrite(right_path, right_img)
        rospy.loginfo(f"保存图像对 {count}: {left_path}, {right_path}")
        count += 1

# 设置近似时间同步，确保同时获得左右图像
sub_left = Subscriber(left_image_topic, Image)
sub_right = Subscriber(right_image_topic, Image)
ats = ApproximateTimeSynchronizer([sub_left, sub_right], queue_size=5, slop=0.2)
ats.registerCallback(image_callback)

rospy.loginfo("Stereo image capture node started, move chessboard into view...")
rospy.spin()
