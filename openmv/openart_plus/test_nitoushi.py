
# 这是一个更强大的逆透视方案
import sensor
import image
import time
import math
import mjpeg
from pyb import LED
from machine import UART
from ulab import numpy as np
import seekfree
import ustruct

# 定义原图中梯形的四个顶点 (x, y)
# 你需要根据实际画面中黄线构成的梯形来手动测量这四个点
# 顺序通常是：左上, 右上, 右下, 左下
src_corners = [(-100, 30), (120, 30), (160, 100), (0, 100)] 

# 定义目标图像（长方形）的四个顶点
dst_corners = [(0, 0), (160, 0), (160, 120), (0, 120)]

# ======================== 常量定义（集中管理，便于修改） ========================
# 串口配置
UART_PORT = 12
UART_BAUDRATE = 115200

# 摄像头配置
CAMERA_PIXFORMAT = sensor.RGB565
CAMERA_FRAMESIZE = sensor.QQVGA  # 160x120
CAMERA_FRAMERATE = 60
CAMERA_BRIGHTNESS = 800

# 屏幕尺寸
SCREEN_WIDTH = 160
SCREEN_HEIGHT = 120
SCREEN_CENTER_X = SCREEN_WIDTH // 2  # 80
SCREEN_CENTER_Y = SCREEN_HEIGHT // 2  # 60

# 卡尔曼滤波配置
KALMAN_MAX_LOST_FRAMES = 10
KALMAN_INIT_POS_X = SCREEN_CENTER_X
KALMAN_INIT_POS_Y = SCREEN_CENTER_Y
MAX_SPEED = 80

# 锁定逻辑配置
LOCK_JUMP_THRESHOLD = 20  # 坐标跳变超过20像素视为同色干扰
LOCK_MAX_LOST_FRAMES = 5  # 丢失5帧解除锁定

# 通信协议常量
PROTOCOL_HEADER1 = 0xA5
PROTOCOL_HEADER2_COORD = 0xA6
PROTOCOL_HEADER2_ANGLE = 0xA7
PROTOCOL_FOOTER = 0x5B

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(CAMERA_PIXFORMAT)
sensor.set_framesize(CAMERA_FRAMESIZE)
sensor.set_framerate(CAMERA_FRAMERATE)
sensor.set_auto_gain(False)  # 关闭自动增益
sensor.set_auto_whitebal(False)  # 关闭自动白平衡
sensor.set_brightness(CAMERA_BRIGHTNESS)
sensor.set_contrast(2) # 对比度
sensor.set_vflip(True)
sensor.skip_frames(time=200)  # 跳过初始帧，让摄像头稳定
sensor.set_hmirror(True)
sensor.skip_frames(time=200)  # 跳过初始帧，让摄像头稳定
clock = time.clock()
LED(4).on()
time.sleep_ms(1000)
LED(4).off()

while(True):
    img = sensor.snapshot()
    
    # 1. 创建一个等大小的空白图像作为“结果画布”
    # 这样可以避免原图干扰
    res_img = image.Image(sensor.width(), sensor.height(), sensor.RGB565)
    
    # 2. 将原图变换后画到新画布上
    res_img.draw_image(img, 0, 0, corners=src_corners, dest_corners=dst_corners)
    
    # 3. 关键：在 IDE 中，最后被 flush 的图像会被显示
    # 我们可以通过把 img 替换掉或者简单的直接操作
    img.replace(res_img) 
    
    # 此时 IDE 右上角就会显示拉直后的 res_img