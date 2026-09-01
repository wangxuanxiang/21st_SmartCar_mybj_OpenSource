import sensor
import image
import time
import math
from pyb import LED
from machine import UART
from ulab import numpy as np
import seekfree
import ustruct

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()
uart = UART(2, baudrate=115200)

H_matrix = [[-9.65659491e-01, -6.04356653e-01, 1.02806428e+02],
            [-7.06519421e-02, 1.19662988e+00, -9.45029305e+01],
            [-5.28740196e-03, -5.25801317e-02, 1.00000000e+00]]
last_angle = 0.0
last_x, last_y = 0.0, 0.0
alpha = 0.7

def pixel_to_real_world(u, v):
        """
        将像素坐标转换为实际物理坐标
        :param u: 像素点的 x 坐标 (列)
        :param v: 像素点的 y 坐标 (行)
        :return: 真实的物理坐标 (X_w, Y_w)
        """
        # 计算缩放因子
        w_prime = H_matrix[2][0] * u + H_matrix[2][1] * v + H_matrix[2][2]
        # 计算真实的物理坐标
        X_w = (H_matrix[0][0] * u + H_matrix[0][1] * v + H_matrix[0][2]) / w_prime
        Y_w = (H_matrix[1][0] * u + H_matrix[1][1] * v + H_matrix[1][2]) / w_prime

        return X_w, Y_w

def send_coordinate_angle(x, y, angle):
        """发送目标坐标（带防抖和范围限制）"""
        # is_first_send = (self.last_sent_x == SCREEN_CENTER_X) and (self.last_sent_y == SCREEN_CENTER_Y)

        # 坐标换算
        x = int(round(x, 1) * 10)
        y = int(round(y, 1) * 10)
        angle = int(round(angle, 1) * 10)

        # 打包并发送数据
        data = ustruct.pack(
            "<BBhhhB",
            0xA1,
            0xA2,
            x,
            y,
            angle,
            0xA3
        )
        uart.write(data)

while(True):
    clock.tick()
    img = sensor.snapshot()
    img.binary([(230, 255)])
    real_center = []
    for blob in img.find_blobs([(255, 255)], pixels_threshold=5, area_threshold=5, merge=True):
        #print(blob.cx(), blob.cy())
        #img.draw_rectangle(blob.rect())
        real_cx, real_cy = pixel_to_real_world(blob.cx(), blob.cy())
        real_center.append((real_cx, real_cy))
        # img.draw_cross(int(blob.cx()), int(blob.cy()), color=127, size=5)

    if len(real_center) == 2:
        # 计算当前现实世界原始的中点和角度
        curr_x = (real_center[0][0] + real_center[1][0]) / 2
        curr_y = (real_center[0][1] + real_center[1][1]) / 2
        curr_angle = math.degrees(math.atan2(real_center[1][1] - real_center[0][1], real_center[1][0] - real_center[0][0]))
        
        # 一阶滞后滤波，防止数据跳变
        target_coordinate_x = alpha * curr_x + (1 - alpha) * last_x
        target_coordinate_y = alpha * curr_y + (1 - alpha) * last_y
        target_angle = alpha * curr_angle + (1 - alpha) * last_angle
        
        # 更新历史数据
        last_x, last_y = target_coordinate_x, target_coordinate_y
        last_angle = target_angle

        send_coordinate_angle(target_coordinate_x, target_coordinate_y, target_angle)
    else:
        data = ustruct.pack("<BBhhhB", 0xA1, 0xA2, -1, -1, -9999, 0xA3)
        uart.write(data)
        pass
    #print(clock.fps())