import argparse
import datetime
import open3d as o3d
import numpy as np


class RecorderWithCallback:

    def __init__(self, config, device, filename, align_depth_to_color):
        # Global flags
        self.flag_exit = False
        self.flag_record = False
        self.filename = filename

        self.align_depth_to_color = align_depth_to_color
        self.recorder = o3d.io.AzureKinectRecorder(config, device)
        if not self.recorder.init_sensor():
            raise RuntimeError('Failed to connect to sensor')

    def escape_callback(self, vis):
        self.flag_exit = True
        if self.recorder.is_record_created():
            print('Recording finished.')
        else:
            print('Nothing has been recorded.')
        return False

    def space_callback(self, vis): # 空格键用于录制交互
        if self.flag_record: # 停止
            print('Recording paused. '
                  'Press [Space] to continue. '
                  'Press [ESC] to save and exit.')
            self.flag_record = False

        elif not self.recorder.is_record_created(): # 开始
            if self.recorder.open_record(self.filename):
                print('Recording started. '
                      'Press [SPACE] to pause. '
                      'Press [ESC] to save and exit.')
                self.flag_record = True

        else: #继续
            print('Recording resumed, video may be discontinuous. '
                  'Press [SPACE] to pause. '
                  'Press [ESC] to save and exit.')
            self.flag_record = True

        return False

    def run(self):
        glfw_key_escape = 256
        glfw_key_space = 32
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.register_key_callback(glfw_key_escape, self.escape_callback)
        vis.register_key_callback(glfw_key_space, self.space_callback)

        vis.create_window('recorder', 1920, 540)
        print("Recorder initialized. Press [SPACE] to start. "
              "Press [ESC] to save and exit.")

        vis_geometry_added = False
        while not self.flag_exit:
            rgbd = self.recorder.record_frame(self.flag_record,
                                              self.align_depth_to_color) # 录制函数，flag_record通过空格键控制
            # if rgbd is not None:
            #     print(rgbd.color.shape)
            if rgbd is None:
                continue
            
            color_image = rgbd.color
            depth_image = rgbd.depth
            print(np.array(depth_image.Image).shape)
            print(type(depth_image.Image))
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image, depth_image,
                convert_rgb_to_intensity=False)
            print(np.array(rgbd_image).shape)

            if not vis_geometry_added:
                vis.add_geometry(rgbd)
                vis_geometry_added = True

            vis.update_geometry(rgbd)
            vis.poll_events()
            vis.update_renderer()

        self.recorder.close_record() # 停止录制，flag_exit通过Esc键控制


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Azure kinect mkv recorder.')
    parser.add_argument('--config', type=str, help='input json kinect config')
    parser.add_argument('--output',default='data/cs.npy', type=str, help='output mkv filename')
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
    # '-a'的'-'代表可选,'store_true'默认False,命令行开启后为True
    args = parser.parse_args()

    if args.list:
        o3d.io.AzureKinectSensor.list_devices()
        exit()

    if args.config is not None:
        config = o3d.io.read_azure_kinect_sensor_config(args.config)
    else:
        config = o3d.io.AzureKinectSensorConfig()

    if args.output is not None:
        filename = args.output
    else:
        filename = '{date:%Y-%m-%d-%H-%M-%S}.mkv'.format(
            date=datetime.datetime.now())
    
    print('Prepare writing to {}'.format(filename))

    device = args.device
    if device < 0 or device > 255:
        print('Unsupported device id, fall back to 0')
        device = 0

    r = RecorderWithCallback(config, device, filename,
                             args.align_depth_to_color)
    r.run()
    
    
    
    # import open3d as o3d

    # # 创建回放器对象
    # playback = o3d.io.AzureKinectPlayback()

    # # 打开录制文件
    # filename = "data/cs.mkv"
    # success = playback.open(filename)

    # if not success:
    #     print(f"Failed to open file {filename}")
    #     exit()

    # # 启动回放
    # while True:
    #     # 获取一帧RGBD数据
    #     rgbd = playback.read_frame()

    #     # 回放结束
    #     if not rgbd:
    #         break
            
    #     # 如果有数据，将RGBD图像显示出来
    #     o3d.visualization.draw_geometries([rgbd])

    # # 关闭回放
    # playback.close()
    
    import cv2
    import numpy as np

    # 定义视频文件路径
    filename = 'data/cs.mkv'

    # 创建一个视频捕获对象
    cap = cv2.VideoCapture(filename)

    # 检查是否成功打开视频文件
    if not cap.isOpened():
        print("Unable to open file:", filename)
        exit()

    # 处理视频文件的每一帧
    frames = []
    while True:
        # 捕获一帧数据
        ret, frame = cap.read()

        # 视频结束
        if not ret:
            break

        # 将数据转换成Numpy数组
        frame = np.array(frame)

        # 将这一帧数据存储到列表中
        frames.append(frame)

    # 关闭视频文件
    cap.release()

    # 打印视频数据的维度
    print(f"Video data shape: {np.array(frames).shape}")
   
