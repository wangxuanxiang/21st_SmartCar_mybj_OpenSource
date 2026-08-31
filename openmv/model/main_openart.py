import sensor
import image
import time
import math
import tf
import mjpeg
from pyb import LED
from machine import UART
from ulab import numpy as np
import seekfree
import ustruct

# ======================== 常量定义 ========================
# 串口配置
UART_PORT = 2
UART_BAUDRATE = 460800

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

# 通信协议常量
PROTOCOL_HEADER1 = 0xA5
PROTOCOL_HEADER2_COORD = 0xA6
PROTOCOL_HEADER2_ANGLE = 0xA7
PROTOCOL_HEADER2_APRILTAG = 0xA8
PROTOCOL_FOOTER = 0x5B

# 颜色标签映射（label → 名称）
LABEL_TO_COLOR = {
    0: 'red',
    1: 'blue',
    2: 'brown',
    3: 'white',
    4: 'green'
}

# 颜色类型ASCII码映射
COLOR_TYPE_MAP = {
    'red': ord('S'),
    'blue': ord('E'),
    'green': ord('T'),
    'brown': ord('B'),
    'white': ord('W'),
    'Apriltag':ord('A'),
    '': 0x00  # 默认值
}

# 绘制颜色映射
DRAW_COLORS = {
    'red': (255, 0, 0),      # 红色沙包
    'green': (0, 255, 0),    # 网球
    'blue': (0, 0, 255),     # 蓝色沙包
    'white': (255, 255, 255),# 白色玩具熊
    'grey': (100, 100, 100), # 卡尔曼框
    'black': (0, 0, 0),      # 锁定标识
    'brown': (150, 75, 0)    # 棕色玩具熊
}

#设置模型路径
face_detect = '/sd/yolo3_iou_smartcar_final_with_post_processing.tflite'

#载入模型
net = tf.load(face_detect)

# ======================== 通信模块 ========================
class Communicator:
    def __init__(self, uart):
        self.uart = uart
        self.last_sent_x = SCREEN_CENTER_X  # 初始化为屏幕中心
        self.last_sent_y = SCREEN_CENTER_Y

    def send_coordinate(self, x, y, obj_type = ''):
        """发送目标坐标（带防抖和范围限制）"""
        # is_first_send = (self.last_sent_x == SCREEN_CENTER_X) and (self.last_sent_y == SCREEN_CENTER_Y)

        # 坐标取整
        x = int(round(x))
        y = int(round(y))

        # 防抖：变化量过小且y坐标在上方时不发送(会造成第一次视觉伺服变迟钝)
        # if not is_first_send and abs(x - self.last_sent_x) < 3 and abs(y - self.last_sent_y) < 3 and y <= 40:
            # return

        # 限制单次坐标变化幅度（最大±30）
        dx_coord = min(30, max(-30, x - self.last_sent_x))
        dy_coord = min(30, max(-30, y - self.last_sent_y))
        x_limited = self.last_sent_x + dx_coord
        y_limited = self.last_sent_y + dy_coord

        # 坐标范围限制（0~160, 0~120）
        x_limited = max(0, min(SCREEN_WIDTH, x_limited))
        y_limited = max(0, min(SCREEN_HEIGHT, y_limited))

        # 更新上次发送的坐标
        self.last_sent_x = x_limited
        self.last_sent_y = y_limited

        # 获取颜色类型对应的ASCII码
        type_char = COLOR_TYPE_MAP.get(obj_type, 0x00)

        # 打包并发送数据
        data = ustruct.pack(
            "<BBBBBB",
            PROTOCOL_HEADER1,
            PROTOCOL_HEADER2_COORD,
            x_limited,
            y_limited,
            type_char,
            PROTOCOL_FOOTER
        )
        self.uart.write(data)

    def send_coordinate_with_angle(self, tag_cx, tag_cy, rotation):
        tag_cx = int(round(tag_cx))
        tag_cy = int(round(tag_cy))
        rotation = int(round(rotation * 10))
        data = ustruct.pack(
            "<BBBBHB",
            PROTOCOL_HEADER1,
            PROTOCOL_HEADER2_APRILTAG,
            tag_cx,
            tag_cy,
            rotation,
            PROTOCOL_FOOTER
        )
        self.uart.write(data)

# ======================== 坐标矫正模块 ========================
class CoordinateCorrection:
    def __init__(self):
        self.tag_family = image.TAG25H9

    def coordinate_correction(self, img):
        """寻找并标记Apriltag， 并计算实际偏转角度"""
        tags = img.find_apriltags(families = self.tag_family)
        tag_cx, tag_cy = None, None

        if tags:
            tag = tags[0]
            tag_cx = tag.cx()
            tag_cy = tag.cy()
            rotation = (math.acos(math.cos(tag.y_rotation()) * math.cos(tag.z_rotation())) - math.pi / 2)
            img.draw_rectangle(tag.rect(), color=(255, 0, 0))
            img.draw_cross(tag_cx, tag_cy, color=(0, 255, 0))

        return (tag_cx, tag_cy, rotation) if tag_cx is not None else None

# ======================== 全局状态变量 ========================

# 模式定义
MODE_CORRECTION = 0      # 坐标校正
MODE_MODEL = 1           # 模型模式
MODE_WAITING = 2         # 等待模式
current_mode = MODE_WAITING

# 时间戳
last_time = time.ticks_ms()

# ======================== 工具函数 ========================
def handle_uart_commands(uart):
    """处理串口命令，切换运行模式"""
    global current_mode
    if uart.any():
        cmd = uart.read(1)
        if cmd == b'C':
            current_mode = MODE_CORRECTION
        elif cmd == b'M':
            current_mode = MODE_MODEL
        elif cmd == b'F':
            current_mode = MODE_WAITING

# ======================== 初始化 ========================
# 串口初始化
uart = UART(UART_PORT, baudrate=UART_BAUDRATE)

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

# LCD初始化
lcd = seekfree.IPS200(3)
lcd.full()

# 创建模块实例
tag_corrector = CoordinateCorrection()
communicator = Communicator(uart)

# ======================== 主循环 ========================
while True:
    clock.tick()
    img = sensor.snapshot()
    # 时间戳更新（计算卡尔曼滤波时间步长）
    current_time = time.ticks_ms()
    delta_time = time.ticks_diff(current_time, last_time)
    Ts = max(delta_time / 1000.0, 0.01)  # 避免Ts过小
    last_time = current_time

    # 处理串口命令
    handle_uart_commands(uart)

    # 等待模式：无操作
    if current_mode == MODE_WAITING:
        continue

    # 坐标校正模式
    elif current_mode == MODE_CORRECTION:
        tag_center = tag_corrector.coordinate_correction(img)
        if tag_center is not None:
            tag_cx, tag_cy, rotation = tag_center
            rotation = (180 * rotation) / math.pi + 90
            communicator.send_coordinate_with_angle(tag_cx, tag_cy, rotation)

    # 模型模式
    elif current_mode == MODE_MODEL:
        is_sent = False # 是否发送了坐标
        center = []
        img1 = img.copy(0.75, 1)
        for obj in tf.detect(net,img1):
            x1,y1,x2,y2,label,scores = obj
            if(scores>0.60):
                x1 = int(x1 * img.width())
                y1 = int(y1 * img.height())
                x2 = int(x2 * img.width())
                y2 = int(y2 * img.height())

                w = x2 - x1
                h = y2 - y1
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                color = LABEL_TO_COLOR[label]
                center.append((cx, cy, color))
                img.draw_rectangle((x1,y1,w,h), color=DRAW_COLORS[color])
                img.draw_cross(cx, cy, color=DRAW_COLORS[color])
        
        if center:
            target = max(center, key=lambda coordinate: coordinate[1])
            target_x = target[0]
            target_y = target[1]
            target_color = target[2]
            communicator.send_coordinate(target_x, target_y, target_color)
            is_sent = True

        displayed_text = 'YES' if is_sent else 'NO'
        displayed_text_color = DRAW_COLORS['green'] if is_sent else DRAW_COLORS['red']
        img.draw_string(5, 5, displayed_text, color = displayed_text_color, scale = 2)

    # 显示图像到LCD
    lcd.show_image(img, SCREEN_WIDTH, SCREEN_HEIGHT, zoom=0)
    # print(clock.fps())
