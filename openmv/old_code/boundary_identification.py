import sensor, image, time, math, mjpeg
from pyb import LED
from machine import UART
from ulab import numpy as np
import ustruct

##########################串口初始化#########################
uart = UART(2, baudrate=115200)
uart.write("uart test\r\n")

##########################摄像头初始化########################
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_framerate(60)
# sensor.set_auto_gain(False) # 自动增益
sensor.set_auto_whitebal(False)
sensor.set_brightness(1000)
# sensor.set_contrast(2) # 对比度
sensor.skip_frames(time = 200)
clock = time.clock()
LED(4).on

#######################变量定义######################
YELLOW_THRESHOLD = (70, 100, -128, 127, 10, 127) # 黄色边界阈值

#######################函数定义######################
def boundary_correction(mode):
    blobs = []
    center = 0
    img = sensor.snapshot()


    if mode == 'row': #行
        num = [0, 26, 52, 80, 106, 132]
    if mode == 'column': #列
        num = [0, 20, 40, 60, 80, 100]
    for x in num:
        if mode == 'row': # 从左到右找色块
            result = img.find_blobs([YELLOW_THRESHOLD], roi = [x,0,26,120] ,pixels_threshold=400, area_threshold=400, margin=1, merge=True, invert=0)
        if mode == 'column':# 从上到下找色块
            result = img.find_blobs([YELLOW_THRESHOLD], roi = [0,x,160,20] ,pixels_threshold=400, area_threshold=400, margin=1, merge=True, invert=0)
        if result:
            result = min(result, key= lambda b: abs(b.area() - 1250))
            blobs.append(result)
            center += 1
            img.draw_rectangle(result.rect(), color = (255, 0, 0), scale = 1, thickness = 1)
        else:
            break
    if center >= 4:
        l = img.get_regression([YELLOW_THRESHOLD])
        if l:
            img.draw_line(l.line(), color = (255, 0, 0), thickness = 2)
            theta = l.theta()
            if theta > 90:
                angle = theta - 180
            else:
                angle = theta
            print(f"{angle}")
        else:
            print("no found")
    else:
        print("insufficient blobs")

while (True):
    boundary_correction('row')
    # boundary_correction('column')
