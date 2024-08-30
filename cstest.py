import sys
import os
cwd = os.getcwd()
sys.path.append(cwd)
# sys.path.insert(1, '../pyKinectAzure/')

import numpy as np
from pyKinectAzure.pyKinectAzure import pyKinectAzure, _k4a
from pyKinectAzure import _k4a
import cv2

# # Path to the module
# # TODO: Modify with the path containing the k4a.dll from the Azure Kinect SDK
# # modulePath = 'C:\\Program Files\\Azure Kinect SDK v1.4.0\\sdk\\windows-desktop\\amd64\\release\\bin\\k4a.dll'
# modulePath = 'C:/Program Files/Azure Kinect SDK v1.4.1/sdk/windows-desktop/amd64/release/bin/k4a.dll'

# def save_npy(color_image_list1,depth_image_list2):
#     a = np.array(color_image_list1)
#     b = np.array(depth_image_list2)

#     np.save('color.npy', a)
#     np.save('depth.npy', b)
# # if __name__ == "__main__":

# 	# # Initialize the library with the path containing the module
# 	# pyK4A = pyKinectAzure(modulePath)

# 	# # Open device
# 	# pyK4A.device_open()

# 	# # Modify camera configuration
# 	# device_config = pyK4A.config
# 	# device_config.color_resolution = _k4a.K4A_COLOR_RESOLUTION_1080P
# 	# device_config.depth_mode = _k4a.K4A_DEPTH_MODE_WFOV_2X2BINNED
# 	# print(device_config)

# 	# # Start cameras using modified configuration
# 	# pyK4A.device_start_cameras(device_config)

# 	# k = 0
# 	# list1=[] #保存RGB图像
# 	# list2=[] #保存深度图像
    
# 	# while True:
# 	# 	# Get capture
# 	# 	pyK4A.device_get_capture()

# 	# 	# Get the color image from the capture
# 	# 	depth_image_handle = pyK4A.capture_get_depth_image()


# 	# 	if depth_image_handle:

# 	# 		# Read and convert the image data to numpy array:
# 	# 		depth_image = pyK4A.image_convert_to_numpy(depth_image_handle)
# 	# 		list2.append(depth_image)
# 	# 		# Convert depth image (mm) to color, the range needs to be reduced down to the range (0,255)
# 	# 		depth_color_image = cv2.applyColorMap(np.round(depth_image/30).astype(np.uint8), cv2.COLORMAP_JET)

# 	# 		# Plot the image
# 	# 		cv2.namedWindow('Colorized Depth Image',cv2.WINDOW_NORMAL)
# 	# 		cv2.imshow('Colorized Depth Image',depth_color_image)
# 	# 		k = cv2.waitKey(25)

# 	# 		# Release the image
# 	# 		pyK4A.image_release(depth_image_handle)

# 	# 	color_image_handle = pyK4A.capture_get_color_image()
# 	# 	# Check the image has been read correctly

# 	# 	if color_image_handle:

# 	# 		# Read and convert the image data to numpy array:
# 	# 		color_image = pyK4A.image_convert_to_numpy(color_image_handle)
# 	# 		list1.append(color_image)
# 	# 		# Plot the image
# 	# 		cv2.namedWindow('Color Image', cv2.WINDOW_NORMAL)
# 	# 		cv2.imshow("Color Image", color_image)
# 	# 		k = cv2.waitKey(20)
# 	# 		# Release the image
# 	# 		pyK4A.image_release(color_image_handle)



# 	# 	pyK4A.capture_release()
		

# 	# 	if k==27:    # Esc key to stop
# 	# 		break
		
# 	# save_npy(list1, list2)
# 	# pyK4A.device_stop_cameras()
# 	# pyK4A.device_close()
 
import argparse
import open3d as o3d

class ViewerWithCallback:

    def __init__(self, config, device, align_depth_to_color):
        self.flag_exit = False
        self.align_depth_to_color = align_depth_to_color

        self.sensor = o3d.io.AzureKinectSensor(config) # 配置相机参数
        if not self.sensor.connect(device):
            raise RuntimeError('Failed to connect to sensor')
        
        self.num = 0

    def escape_callback(self, vis):
        self.flag_exit = True
        return False
    
    def save_callback(self, vis): # 提取RGB图像和深度图像并保存，按's'键保存
        # num = np.random.randint(100)
        # print(self.rgbd.color)
        
        o3d.io.write_image('data/color/color'+str(self.num)+'.png', self.rgbd.color, quality = - 1)
        o3d.io.write_image('data/depth/depth'+str(self.num)+'.png', self.rgbd.depth, quality = - 1)
        self.num += 1
        return False
    
    def save_npy(self, color_image_list1,depth_image_list2):
        a = np.asarray(color_image_list1)
        b = np.asarray(depth_image_list2)
        print(a)
        print(b)
        np.save('color.npy', a)
        np.save('depth.npy', b)
  

    def run(self):
        glfw_key_escape = 256
        glfw_key_save = 83
        vis = o3d.visualization.VisualizerWithKeyCallback() # 定义可视化类
        vis.register_key_callback(glfw_key_escape, self.escape_callback) # 注册退出回调函数
        vis.register_key_callback(glfw_key_save, self.save_callback) # 注册保存回调函数
        vis.create_window('viewer', 1920, 540) # 创建窗口
        print("Sensor initialized. Press [ESC] to exit.")

        vis_geometry_added = False
        k = 0
        list1=[] #保存RGB图像
        list2=[] #保存深度图像
        while not self.flag_exit:
            self.rgbd = self.sensor.capture_frame(self.align_depth_to_color) # 采集图像
            print(self.align_depth_to_color)
            if self.rgbd is None:
                continue

            if not vis_geometry_added:
                vis.add_geometry(self.rgbd)
                vis_geometry_added = True
            
            o3d.io.write_image('data/color/color' + str(self.num)+'.png', self.rgbd.color, quality = - 1)
            o3d.io.write_image('data/depth/depth' + str(self.num)+'.png', self.rgbd.depth, quality = - 1)
            self.num += 1
            # vis.capture_screen_image('b.png', do_render=False).shape
            vis.update_geometry(self.rgbd)
            color_image = self.rgbd.color
            
            print(np.array(color_image).shape)
            depth_image = self.rgbd.depth
            print(np.array(depth_image).shape)
            
            vis.poll_events()
            vis.update_renderer()
            
            
            
            # color_image = self.rgbd.color
            # depth_image = self.rgbd.depth
            # color_image = o3d.geometry.Image(color_image)
            # depth_image = o3d.geometry.Image(depth_image)
            # list1.append(color_image)
            # list2.append(depth_image)
            
            # k = cv2.waitKey(25)
            # if k == 27:  # Esc
            #     break
        # self.save_npy(list1, list2)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Azure kinect mkv recorder.')
    parser.add_argument('--config', type=str, help='input json kinect config')
    parser.add_argument('--list',
                        action='store_true',
                        help='list available azure kinect sensors')
    parser.add_argument('--device',
                        type=int,
                        default=0,
                        help='input kinect device id')
    parser.add_argument('-a',
                        '--align_depth_to_color',
                        default= True,
                        action='store_true',
                        help='enable align depth image to color')
    args = parser.parse_args()

    if args.list:
        o3d.io.AzureKinectSensor.list_devices()
        exit()

    if args.config is not None:
        config = o3d.io.read_azure_kinect_sensor_config(args.config)
    else:
        config = o3d.io.AzureKinectSensorConfig()

    device = args.device
    if device < 0 or device > 255:
        print('Unsupported device id, fall back to 0')
        device = 0

    v = ViewerWithCallback(config, device, args.align_depth_to_color)
    v.run()






