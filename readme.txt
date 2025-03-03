1. 启动 ROS Master 和相机驱动：确保 ROS 主节点已运行（roscore），并启动您的左右相机节点以发布图像。例如，如果使用USB摄像头，可运行两个摄像头驱动使图像发布在/left/image_raw和/right/image_raw话题下。

2. 运行图像采集节点：执行上述图像采集节点脚本，例如：

rosrun <your_package> stereo_image_capture.py
将棋盘格标定板放入两相机视野中，不同角度位置采集多组图像。程序检测到棋盘格后会自动保存左右图像对到~/stereo_calib_images文件夹，并在终端输出如下信息：

[INFO] [....]: 保存图像对 0: /home/user/stereo_calib_images/left_0.png, .../right_0.png  
[INFO] [....]: 保存图像对 1: /home/user/stereo_calib_images/left_1.png, .../right_1.png  
...  
每成功检测一次棋盘格并保存图像对，终端会增加一条日志。采集完毕后按Ctrl+C结束节点。确保采集不同方位至少10对以上图像以提高标定精度。

3.运行相机标定脚本：在采集完图像后，执行相机标定代码脚本（可用rosrun运行，亦可直接用Python运行）。例如：

rosrun <your_package> stereo_calibrate.py
脚本将读取保存的图像进行标定，并在控制台打印结果，例如摄像机的单目重投影误差、立体标定误差和相机基线距离：

单目标定完成：左相机误差=0.2341像素，右相机误差=0.2108像素  
立体标定完成：重投影误差=0.4215像素  
相机基线距离: 0.1205 米  
标定参数已保存至 left.yaml 和 right.yaml  

标定完成后，将在当前目录生成left.yaml和right.yaml文件，其中包含左右相机内参、畸变系数、校正矩阵和投影矩阵等参数。


4.发布相机内参：使用相机参数发布节点将标定结果发布为 ROS 话题：

rosrun <your_package> camera_info_publisher.py  
启动后，该节点会将标定参数发布到/left/camera_info和/right/camera_info（默认）话题，并输出日志确认：

[INFO] [....]: 已发布左相机和右相机 CameraInfo 参数.  
此步骤保证其他ROS节点能够获取相机的标定参数用于图像校正和深度计算。如果使用ROS自带的stereo_image_proc节点进行立体校正和视差计算，可以跳过步骤4和5，直接运行stereo_image_proc（其会自动订阅上述CameraInfo和图像话题）。

5.运行立体匹配节点：确保相机图像流和CameraInfo话题都在发布，然后运行立体匹配代码：

rosrun <your_package> stereo_matching.py  
该节点启动后，会订阅左右相机图像及CameraInfo，使用OpenCV校正图像并计算视差和深度。运行时会弹出一个窗口实时显示计算的视差图（灰度表示视差大小），每秒在控制台打印一次中心点的深度值。例如：

[INFO] [....]: 中心像素深度约为 1.247 米  
[INFO] [....]: 中心像素深度约为 1.256 米  
...  
您可以将物体置于两相机视场中央来验证深度变化。按下Ctrl+C可停止节点。


以上步骤完成后，您将获得左右相机的标定参数，以及实时计算的视差图和深度信息。您可以根据需要修改算法参数（例如调整StereoSGBM的numDisparities和blockSize以适应不同基线或分辨率），以及使用RViz或rqt_image_view等工具可视化结果（例如查看发布的视差图像）。该流程提供了从图像采集、相机标定到深度计算的完整示例。