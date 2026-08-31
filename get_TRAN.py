import cv2
import numpy as np

# 1. 图像中的四个点坐标 (pixel)
'''
pts_src = np.array([[3, 23], [157, 23], [5, 95], [156, 96]], dtype='float32')

# 2. 对应的物理世界坐标 (cm)
pts_dst = np.array([[60, 60], [-56,60], [17, 5], [-16, 5]], dtype='float32')

# 3. 计算单应性矩阵 H
h, status = cv2.findHomography(pts_src, pts_dst)
print(h)
'''
"""
主车摄像头
[[-1.91770577e+01 -3.33345955e-01  1.42802747e+03]
 [ 2.31947497e-01 -1.52060135e+01  1.98827669e+03]
 [-1.00738938e-02  8.54771435e-01  1.00000000e+00]]
"""
"""
从车摄像头
[[-3.47719666e+00  7.36005730e-03  2.85357508e+02]
 [ 2.53077460e-02 -2.72395850e+00  3.37670322e+02]
 [ 4.21795767e-04  1.55811070e-01  1.00000000e+00]]
"""
def pixel_to_real_world(u, v, H):
    '''
    将像素坐标转换为实际物理坐标
    :param u: 像素点的 x 坐标 (列)
    :param v: 像素点的 y 坐标 (行)
    :param H: 3x3 的单应性矩阵 (由 cv2.findHomography 求得)
    :return: 真实的物理坐标 (X_w, Y_w)
    '''
    # 1. 构造像素点的齐次坐标向量 [u, v, 1]
    pixel_point = np.array([u, v, 1.0])

    # 2. 矩阵乘法：H 乘以 像素向量
    # result 是一个包含三个元素的向量 [x', y', w']
    result = np.dot(H, pixel_point)

    # 3. 提取 w' (即缩放因子)
    w_prime = result[2]

    # 防止除以 0 的异常情况
    if w_prime == 0:
        print("警告: 缩放因子为0，无法转换")
        return None, None

    # 4. 归一化：除以 w' 得到最终物理坐标
    X_w = result[0] / w_prime
    Y_w = result[1] / w_prime

    return X_w, Y_w

#'''
# ================= 测试代码 =================

# 假设这是你之前用 cv2.findHomography 求出的 3x3 矩阵
# (这里随便填了一组数据作为演示，你需要替换成你真实求出的矩阵)
H_matrix = np.array(
[[-3.47719666e+00  ,7.36005730e-03  ,2.85357508e+02],
 [ 2.53077460e-02 ,-2.72395850e+00  ,3.37670322e+02],
 [ 4.21795767e-04  ,1.55811070e-01  ,1.00000000e+00]]
)

# 假设摄像头画面中识别到了物体，中心像素点坐标为 (160, 120)
target_u = 43
target_v = 14

# 调用函数进行转换
real_x, real_y = pixel_to_real_world(target_u, target_v, H_matrix)

if real_x is not None:
    print(f"目标像素点: ({target_u}, {target_v})")
    print(f"小车 X 方向偏移量 (横向): {real_x:.2f} cm")
    print(f"小车 Y 方向偏移量 (纵向): {real_y:.2f} cm")
    # 如果你需要知道直线距离：
    distance = np.sqrt(real_x ** 2 + real_y ** 2)
    print(f"物体距离小车的直线距离: {distance:.2f} cm")
