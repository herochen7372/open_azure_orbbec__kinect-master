
0. Azure Kinect SDK 官网
https://learn.microsoft.com/zh-cn/azure/kinect-dk/azure-kinect-viewer?source=recommendations

https://www.orbbec.com.cn/index/Sdkdoc/info.html?cate=117&id=42

1. 下载 Azure Kinect viewer v 1.4.2
https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md

https://github.com/orbbec/OrbbecSDK-K4A-Wrapper

2.运行main.py，函数display_all()显示RGB图，深度图和点云图，并保存为.npy文件；
track()函数获取RGB图像并使用KCF算法对框选目标进行跟踪。
kcf_tracking.py实现kcf算法的目标跟踪；plot3dUtils.py是绘制点云图；三个.npy文件分别保存了RGB、深度图以及点云图的信息，read_npy.py文件就是读取这三个文件并显示图像
