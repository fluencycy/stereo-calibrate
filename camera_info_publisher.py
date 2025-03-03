#!/usr/bin/env python3
import rospy
import yaml
from sensor_msgs.msg import CameraInfo

# 初始化ROS节点
rospy.init_node('camera_info_publisher', anonymous=True)

# 获取参数文件路径和发布的话题名称
left_yaml_path = rospy.get_param('~left_yaml', 'left.yaml')
right_yaml_path = rospy.get_param('~right_yaml', 'right.yaml')
left_info_topic = rospy.get_param('~left_camera_info_topic', '/left/camera_info')
right_info_topic = rospy.get_param('~right_camera_info_topic', '/right/camera_info')

# 从YAML文件加载相机参数
def load_camera_info(yaml_file):
    with open(yaml_file, "r") as file:
        calib_data = yaml.safe_load(file)
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    return camera_info_msg

left_info = load_camera_info(left_yaml_path)
right_info = load_camera_info(right_yaml_path)
left_info_pub = rospy.Publisher(left_info_topic, CameraInfo, latch=True, queue_size=1)
right_info_pub = rospy.Publisher(right_info_topic, CameraInfo, latch=True, queue_size=1)

# 设置frame_id（如果需要可修改，与图像消息的frame_id对应）
left_frame_id = rospy.get_param('~left_frame_id', 'left_camera')
right_frame_id = rospy.get_param('~right_frame_id', 'right_camera')
left_info.header.frame_id = left_frame_id
right_info.header.frame_id = right_frame_id

# 发布CameraInfo并保持节点运行
left_info_pub.publish(left_info)
right_info_pub.publish(right_info)
rospy.loginfo("已发布左相机和右相机 CameraInfo 参数.")
rospy.spin()
