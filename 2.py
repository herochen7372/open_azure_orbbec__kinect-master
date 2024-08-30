import open3d as o3d
import numpy as np
# 使用Azure Kinect相机捕获一帧RGBD图像
k4a = o3d.io.AzureKinectSensor(o3d.io.AzureKinectSensorConfig())
rgbd = k4a.capture_frame(True)

# 获取颜色图像数据
color_img = np.array(rgbd.color)

# 显示颜色图像
o3d.visualization.draw_geometries([rgbd])