#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer

rospy.init_node('stereo_matching', anonymous=True)
bridge = CvBridge()

# 读取ROS参数中的话题名称
left_image_topic = rospy.get_param('~left_image_topic', '/left/image_raw')
right_image_topic = rospy.get_param('~right_image_topic', '/right/image_raw')
left_info_topic = rospy.get_param('~left_camera_info_topic', '/left/camera_info')
right_info_topic = rospy.get_param('~right_camera_info_topic', '/right/camera_info')

# 等待相机内参消息（由步骤3发布）
rospy.loginfo("等待相机内参消息...")
left_info_msg = rospy.wait_for_message(left_info_topic, CameraInfo)
right_info_msg = rospy.wait_for_message(right_info_topic, CameraInfo)
rospy.loginfo("已接收到相机内参。")

# 提取标定参数用于校正和视差计算
K1 = np.array(left_info_msg.K).reshape(3,3)
D1 = np.array(left_info_msg.D)
K2 = np.array(right_info_msg.K).reshape(3,3)
D2 = np.array(right_info_msg.D)
R1 = np.array(left_info_msg.R).reshape(3,3)
R2 = np.array(right_info_msg.R).reshape(3,3)
P1 = np.array(left_info_msg.P).reshape(3,4)
P2 = np.array(right_info_msg.P).reshape(3,4)
# Q矩阵需要使用stereoRectify的结果；这里可由P2推算baseline和由K取焦距近似构造。
# 简单起见，也可以在标定时保存并通过参数服务器提供Q。此处假设未畸变图像大小与标定图像相同。
# 计算图像校正映射
image_size = (left_info_msg.width, left_info_msg.height)
left_map1, left_map2 = cv2.initUndistortRectifyMap(K1, D1, R1, P1[:,:3], image_size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(K2, D2, R2, P2[:,:3], image_size, cv2.CV_16SC2)

# 配置StereoSGBM立体匹配参数
max_disp = 64  # 最大视差（像素），需为16的倍数
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=max_disp,
    blockSize=7,
    P1=8 * 3 * 7**2,
    P2=32 * 3 * 7**2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32
)

frame_count = 0
def stereo_callback(left_img_msg, right_img_msg):
    global frame_count
    # 转换ROS图像为OpenCV格式并校正
    left_img = bridge.imgmsg_to_cv2(left_img_msg, "bgr8")
    right_img = bridge.imgmsg_to_cv2(right_img_msg, "bgr8")
    gray_left = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
    rect_left = cv2.remap(gray_left, left_map1, left_map2, cv2.INTER_LINEAR)
    rect_right = cv2.remap(gray_right, right_map1, right_map2, cv2.INTER_LINEAR)
    # 计算视差图
    disparity = stereo.compute(rect_left, rect_right).astype(np.int16)
    # 将视差转换为浮点型并计算深度（Z值）
    disparity_float = disparity.astype(np.float32) / 16.0
    # Q矩阵由cv2.stereoRectify计算得到，这里简化假设已知
    # 利用P2推算Q矩阵（P2[0,3] = -fx*Tx，即 -fx*baseline）
    fx = P1[0,0]
    cx = P1[0,2]; cy = P1[1,2]
    Tx = P2[0,3] / (-fx)  # Baseline 距离（米）
    # 构造Q矩阵
    Q = np.array([[1, 0,   0, -cx],
                  [0, 1,   0, -cy],
                  [0, 0,   0,  fx],
                  [0, 0, -1/Tx, 0]])
    points_3d = cv2.reprojectImageTo3D(disparity_float, Q)
    depth_map = points_3d[:, :, 2]  # 提取深度（Z轴距离）

    # 可视化视差图
    disp_vis = cv2.normalize(disparity_float, None, 0, 255, cv2.NORM_MINMAX)
    disp_vis = np.uint8(disp_vis)
    cv2.imshow("Disparity", disp_vis)
    cv2.waitKey(1)

    # 每隔一定帧打印中心像素深度作为示例
    frame_count += 1
    if frame_count % 30 == 0:  # 大约每30帧打印一次
        h, w = depth_map.shape
        center_depth = depth_map[h//2, w//2]
        rospy.loginfo(f"中心像素深度约为 {center_depth:.3f} 米")

# 同步订阅左右图像话题
left_sub = Subscriber(left_image_topic, Image)
right_sub = Subscriber(right_image_topic, Image)
ats = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=5, slop=0.1)
ats.registerCallback(stereo_callback)

rospy.loginfo("立体匹配节点已启动，正在订阅图像话题...")
rospy.spin()
cv2.destroyAllWindows()
