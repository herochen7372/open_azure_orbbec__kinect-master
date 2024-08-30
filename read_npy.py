import numpy as np
import cv2
import plot3dUtils
colormap = np.load('data/color.npy')    #使用numpy载入npy文件
depthmap=np.load('data/depth.npy')
pointsmap=np.load('data/points.npy')
# print(colormap)
# colormaplist=colormap.tolist()

# i=0
# open3dVisualizer = plot3dUtils.Open3dVisualizer()
# while (i<=len(colormap)-1):

#         open3dVisualizer(pointsmap[i])
#         cv2.namedWindow(' Color Image', cv2.WINDOW_NORMAL)
#         cv2.imshow(' Color Image', colormap[i])
#         cv2.namedWindow(' Depth Image', cv2.WINDOW_NORMAL)
#         cv2.imshow(' Depth Image', depthmap[i])

#         i+=1
#         k = cv2.waitKey(25)
#         if k == 27:  # Esc
#             break


print(colormap.shape)
print(depthmap.shape)
print(pointsmap.shape)
for i in range(512):
    print(depthmap[0,i,i,:])





