from machine import UART
import pyb
import struct
from pyb import LED
import sensor, image, time, math, tf
import os


red = LED(1)
green = LED(2)
blue = LED(3)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_framerate(60)
sensor.set_auto_gain(False)  # 关闭自动增益
sensor.set_auto_whitebal(False)  # 关闭自动白平衡
sensor.set_brightness(600)
sensor.set_contrast(2) # 对比度
sensor.skip_frames(time=2000)  # 跳过初始帧，让摄像头稳定
clock = time.clock()


save_img_num = 0
count = -100

while(True):
    img = sensor.snapshot()             # 获取一幅图像
    # blue.toggle()                       # 蓝灯翻转

    count += 1

    if (count % 20) == 0 and count > 0:
        # 修改文件名称，准备保存
        save_img_num += 1
        image_pat = "/sd/picture/"+str(save_img_num)+"_7_16.jpg"

        # 将拷贝之后的图像保存到sd卡
        img.save(image_pat,quality=99)
