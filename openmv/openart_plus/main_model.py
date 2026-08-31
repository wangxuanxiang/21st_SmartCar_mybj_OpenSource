import sensor
import image
import time
import math
import tf
import mjpeg
from machine import UART
from ulab import numpy as np
import seekfree
import ustruct
from pyb import LED #导入LED

# ======================== 常量定义 ========================
white = LED(4)  # 定义一个LED4   照明灯

# 串口配置
UART_PORT = 12
UART_BAUDRATE = 115200

# 摄像头配置
CAMERA_PIXFORMAT = sensor.RGB565
CAMERA_FRAMESIZE = sensor.QQVGA  # 160x120
CAMERA_FRAMERATE = 60
CAMERA_BRIGHTNESS = 600 # 800在plus上可能导致apriltag识别失败，降低亮度可以改善

# 屏幕尺寸
SCREEN_WIDTH = 160
SCREEN_HEIGHT = 120
SCREEN_CENTER_X = SCREEN_WIDTH // 2  # 80
SCREEN_CENTER_Y = SCREEN_HEIGHT // 2  # 60

# 卡尔曼滤波配置
KALMAN_MAX_LOST_FRAMES = 3
MAX_SPEED = 80
JUMP_KALMAN_THRESHOLD = 30  # 卡尔曼预测跳变超过30像素视为异常

# # 锁定逻辑配置
# LOCK_JUMP_THRESHOLD = 20  # 坐标跳变超过20像素视为干扰
# LOCK_MAX_LOST_FRAMES = 3  # 丢失3帧解除锁定

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
# face_detect = '/sd/yolov4_tiny_final_with_post_processing.tflite'
#载入模型
net = tf.load(face_detect)

global_counter = 0
first_time = time.ticks_ms()

# ======================== 通信模块 ========================
class Communicator:
    def __init__(self, uart):
        self.uart = uart
        # self.last_sent_x = SCREEN_CENTER_X  # 初始化为屏幕中心
        # self.last_sent_y = SCREEN_CENTER_Y

    def send_coordinate(self, x, y, obj_type = ''):
        """发送目标坐标（带防抖和范围限制）"""
        # is_first_send = (self.last_sent_x == SCREEN_CENTER_X) and (self.last_sent_y == SCREEN_CENTER_Y)

        # 坐标取整
        x = int(round(x))
        y = int(round(y))

        # 防抖：变化量过小且y坐标在上方时不发送(会造成第一次视觉伺服变迟钝)
        # if not is_first_send and abs(x - self.last_sent_x) < 3 and abs(y - self.last_sent_y) < 3 and y <= 40:
            # return

        # # 限制单次坐标变化幅度（最大±30）
        # dx_coord = min(30, max(-30, x - self.last_sent_x))
        # dy_coord = min(30, max(-30, y - self.last_sent_y))
        # x_limited = self.last_sent_x + dx_coord
        # y_limited = self.last_sent_y + dy_coord

        # # 坐标范围限制（0~160, 0~120）
        # x_limited = max(0, min(SCREEN_WIDTH, x_limited))
        # y_limited = max(0, min(SCREEN_HEIGHT, y_limited))

        # # 更新上次发送的坐标
        # self.last_sent_x = x_limited
        # self.last_sent_y = y_limited

        # 获取颜色类型对应的ASCII码
        type_char = COLOR_TYPE_MAP.get(obj_type, 0x00)

        # 打包并发送数据
        
        data = ustruct.pack(
            "<BBBBBB",
            PROTOCOL_HEADER1,
            PROTOCOL_HEADER2_COORD,
            x,
            y,
            type_char,
            PROTOCOL_FOOTER
        )
        self.uart.write(data)
        # self.uart.write(f"{PROTOCOL_HEADER1} {PROTOCOL_HEADER2_COORD} {x} {y} {PROTOCOL_FOOTER}\r\n")

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

# ======================== 卡尔曼跟踪模块 ========================
class KalmanTracker:
    MAX_LOST_FRAMES = KALMAN_MAX_LOST_FRAMES

    def __init__(self):
        # 观测矩阵C
        self.C = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]
        ], dtype=np.float)

        # 测量噪声R
        self.R = np.diag([0.5, 0.5, 1.5, 1.5, 2, 2])

        # 初始化状态
        self.reset()

    def reset(self):
        """重置跟踪器状态"""
        self.first_detected = False
        self.lost_count = 0
        self.last_cx, self.last_cy = SCREEN_CENTER_X, SCREEN_CENTER_Y  # 初始中心坐标
        # 初始状态x_hat：[x, y, w, h, vx, vy]
        self.x_hat = np.array([SCREEN_CENTER_X, SCREEN_CENTER_Y, 30, 30, 2, 2], dtype=np.float)
        # 初始协方差矩阵p
        self.p = np.diag([100.0, 100.0, 50.0, 50.0, 300.0, 300.0])

    def kalman_filter(self, Z, Ts, is_detected):
        """
        卡尔曼滤波核心逻辑
        Z: 观测值数组 [cx, cy, w, h, dx, dy]（None表示丢失）
        Ts: 时间步长（秒）
        is_detected: 是否检测到目标
        return: 更新后的状态 [x, y, w, h, vx, vy]
        """
        # 动态设置Q值和阻尼系数
        if is_detected:
            Q_value = [0.02, 0.02, 0.02, 0.02, 0.2, 0.2]
            damping = 0.98
            self.lost_count = 0
        else:
            Q_value = [1.0, 1.0, 0.5, 0.5, 2.0, 2.0]
            damping = 0.9
            self.lost_count += 1
        Q = np.diag(Q_value)

        # 状态转移矩阵A
        A = np.array([
            [1, 0, 0, 0, Ts, 0],
            [0, 1, 0, 0, 0, Ts],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, damping, 0],
            [0, 0, 0, 0, 0, damping]
        ], dtype=np.float)

        # 预测阶段
        x_hat_minus = np.dot(A, self.x_hat)
        p_minus = np.dot(A, np.dot(self.p, A.T)) + Q

        # 更新阶段（仅检测到目标时）
        if is_detected:
            S = np.dot(np.dot(self.C, p_minus), self.C.T) + self.R
            S_reg = S + 1e-4 * np.eye(S.shape[0])  # 防矩阵奇异
            try:
                S_inv = np.linalg.inv(S_reg)
            except np.linalg.LinAlgError:
                # 矩阵奇异时，用对角逆替代（降级方案），新增极小值避免除0
                diag_vals = []
                for i in range(S_reg.shape[0]):
                    val = S_reg[i,i]
                    # 若对角线元素为0/极小值，用1e-6替代，避免除0报错
                    diag_vals.append(1.0 / val if abs(val) > 1e-6 else 1e6)
                S_inv = np.diag(diag_vals)
            K = np.dot(np.dot(p_minus, self.C.T), S_inv)
            self.x_hat = x_hat_minus + np.dot(K, (Z - np.dot(self.C, x_hat_minus)))
            self.p = np.dot((np.eye(6) - np.dot(K, self.C)), p_minus)
        else:
            self.x_hat = x_hat_minus
            self.p = p_minus

        return self.x_hat
    
# ======================== 模型检测模块  ========================
class ModelDetector:
    def __init__(self, net):
        self.net = net

    def detect(self, img):
        """模型检测，返回检测结果列表"""
        img1 = img.copy(0.75, 1)
        return tf.detect(self.net, img1)
    
    # def detect_and_draw(self, img, center):    
    #     img1 = img.copy(0.75, 1)
    #     for obj in tf.detect(self.net, img1):
    #             x1,y1,x2,y2,label,scores = obj
    #             if(scores>0.60):
    #                 x1 = int(x1 * img.width())
    #                 y1 = int(y1 * img.height())
    #                 x2 = int(x2 * img.width())
    #                 y2 = int(y2 * img.height())

    #                 w = x2 - x1
    #                 h = y2 - y1
    #                 cx = (x1 + x2) // 2
    #                 cy = (y1 + y2) // 2
    #                 color = LABEL_TO_COLOR[label]
    #                 center.append((cx, cy, color))
    #                 img.draw_rectangle((x1,y1,w,h), color=DRAW_COLORS[color])
    #                 img.draw_cross(cx, cy, color=DRAW_COLORS[color])
    #     return center
    
    def process_kalman_color(self, img, objects, tracker, color, Ts, center_list, kalman_coords_dict):
        """封装卡尔曼处理单颜色逻辑（棕/白/蓝通用）"""
        detected = False
        target_object = None
        
        if objects:
            target_object = max(objects, key=lambda obj: (obj[2] - obj[0]) * img.width() * (obj[3] - obj[1]) * img.height())  # 选择面积最大的目标
            x1,y1,x2,y2,label,scores = target_object
            x1 = int(x1 * img.width())
            y1 = int(y1 * img.height())
            x2 = int(x2 * img.width())
            y2 = int(y2 * img.height())
            w = x2 - x1
            h = y2 - y1
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            jump_too_large  = False
            if tracker.first_detected:
                distance_squared = (cx - tracker.last_cx)**2 + (cy - tracker.last_cy)**2
                if distance_squared > (JUMP_KALMAN_THRESHOLD**2):
                    jump_too_large = True

            if not jump_too_large:
                # 计算速度
                dx_raw = (cx - tracker.last_cx) / Ts
                dx = max(-MAX_SPEED, min(MAX_SPEED, dx_raw))
                dy_raw = (cy - tracker.last_cy) / Ts
                dy = max(-MAX_SPEED, min(MAX_SPEED, dy_raw))

                # 构建6维新观测值Z
                Z = np.array([cx, cy, w, h, dx, dy], dtype = np.float)

                # 第一次检测到目标时初始化卡尔曼
                if not tracker.first_detected:
                    tracker.x_hat = np.array([cx, cy, w, h, 0, 0], dtype=np.float)
                    tracker.p = np.diag([10.0, 10.0, 5.0, 5.0, 100.0, 100.0])
                    tracker.first_detected = True

                # 卡尔曼滤波更新
                tracker.kalman_filter(Z, Ts, True)
                tracker.last_cx = cx
                tracker.last_cy = cy
                detected = True

                # 绘制原始检测框
                img.draw_rectangle((x1, y1, w, h), color=DRAW_COLORS[color])
                img.draw_cross(cx, cy, color=DRAW_COLORS[color])
            else:
                detected = False  # 跳变过大视为无检测，进入预测阶段
        # 无检测时卡尔曼预测
        if not detected and tracker.first_detected:
            if tracker.lost_count < tracker.MAX_LOST_FRAMES:
                tracker.kalman_filter(None, Ts, is_detected=False)
            else:
                tracker.reset()

        # 绘制卡尔曼预测框并更新坐标
        if tracker.first_detected:
            kcx, kcy = int(tracker.x_hat[0]), int(tracker.x_hat[1])
            kw, kh = max(1, int(tracker.x_hat[2])), max(1, int(tracker.x_hat[3]))
            img.draw_rectangle(kcx - kw//2, kcy - kh//2, kw, kh, color=DRAW_COLORS['grey'])
            img.draw_cross(kcx, kcy, color=DRAW_COLORS['grey'])
            center_list.append((kcx, kcy, color))
            kalman_coords_dict[color] = (kcx, kcy)
        else:
            kalman_coords_dict[color] = (SCREEN_CENTER_X, SCREEN_CENTER_Y)
        
        return target_object

    def draw_other_objects(self, img, objects, center_list):
        """封装绘制其他颜色色块逻辑"""
        for item in objects:
            obj, color_name = item
            x1,y1,x2,y2,label,scores = obj
            x1 = int(x1 * img.width())
            y1 = int(y1 * img.height())
            x2 = int(x2 * img.width())
            y2 = int(y2 * img.height())
            w = x2 - x1
            h = y2 - y1
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # 绘制
            img.draw_rectangle((x1, y1, w, h), color=DRAW_COLORS[color_name])
            img.draw_cross(cx, cy, color=DRAW_COLORS[color_name])
            center_list.append((cx, cy, color_name))

# # ======================== 锁定逻辑模块 ========================
# class TargetLocker:
#     def __init__(self, jump_threshold, max_lost_frames):
#         self.is_locked = False        # 是否锁定目标
#         self.last_cx = SCREEN_CENTER_X    # 上一帧锁定目标的x坐标
#         self.last_cy = SCREEN_CENTER_Y    # 上一帧锁定目标的y坐标
#         self.lost_count = 0           # 锁定目标丢失帧数
#         self.JUMP_THRESHOLD = jump_threshold
#         self.MAX_LOST_FRAMES = max_lost_frames
#         self.locked_kind = ""
#         self.locked_cx = SCREEN_CENTER_X
#         self.locked_cy = SCREEN_CENTER_Y

#     def reset(self):
#         """重置锁定状态，所有变量彻底清零"""
#         self.is_locked = False
#         self.last_cx = SCREEN_CENTER_X
#         self.last_cy = SCREEN_CENTER_Y
#         self.lost_count = 0
#         self.locked_kind = ""
#         self.locked_cx = SCREEN_CENTER_X
#         self.locked_cy = SCREEN_CENTER_Y

#     def is_jump_too_large(self, cx, cy):
#         """判断坐标跳变是否过大（同种类干扰）"""
#         squared_distance = (cx - self.last_cx)**2 + (cy - self.last_cy)**2
#         return squared_distance > (self.JUMP_THRESHOLD**2)

#     def process_lock(self, objects, kalman_coords):
#         target_pos = None
#         locked_object = None
#         target_kind = ""

#         if objects:
#             if not self.is_locked:
#                 # 未锁定 → 选择最下方目标
#                 max_y_obj = max(objects, key=lambda item: (item[1] + item[3]) // 2)
#                 self.locked_kind = LABEL_TO_COLOR[max_y_obj[4]]
                
#                 # 优先使用卡尔曼坐标（如果可用且合理）
#                 if self.locked_kind in ['brown', 'white'] and self.locked_kind in kalman_coords:
#                     kcx, kcy = kalman_coords[self.locked_kind]
#                     # 检查卡尔曼坐标是否合理（不是默认中心值）
#                     if (kcx, kcy) != (SCREEN_CENTER_X, SCREEN_CENTER_Y):
#                         self.locked_cx, self.locked_cy = kcx, kcy
#                     else:
#                         # 卡尔曼坐标不可用，使用原始坐标
#                         self.locked_cx, self.locked_cy = (max_y_obj[0] + max_y_obj[2]) // 2, (max_y_obj[1] + max_y_obj[3]) // 2
#                 else:
#                     # 非卡尔曼滤波物体直接使用原始坐标
#                     self.locked_cx, self.locked_cy = (max_y_obj[0] + max_y_obj[2]) // 2, (max_y_obj[1] + max_y_obj[3]) // 2
                
#                 self.last_cx, self.last_cy = self.locked_cx, self.locked_cy
#                 self.is_locked = True
#                 self.lost_count = 0

#                 target_pos = (self.locked_cx, self.locked_cy)
#                 target_kind = self.locked_kind
#                 locked_object = max_y_obj
#             else:
#                 # 已锁定 → 筛选同色且不跳变的目标
#                 same_color = [o for o in objects if LABEL_TO_COLOR[o[4]] == self.locked_kind]
#                 valid_blobs = []
#                 for obj in same_color:
#                     cx = (obj[0] + obj[2]) // 2
#                     cy = (obj[1] + obj[3]) // 2
                    
#                     # 对于卡尔曼滤波物体，使用卡尔曼坐标判断跳变
#                     if self.locked_kind in ['brown', 'white']:
#                         if self.locked_kind in kalman_coords:
#                             kcx, kcy = kalman_coords[self.locked_kind]
#                             if not self.is_jump_too_large(kcx, kcy):
#                                 valid_blobs.append((obj, cx, cy))
#                     else:
#                         # 非卡尔曼滤波物体使用原始坐标
#                         if not self.is_jump_too_large(cx, cy):
#                             valid_blobs.append((obj, cx, cy))

#                 if valid_blobs:
#                     best_obj, best_cx, best_cy = min(valid_blobs,
#                         key=lambda item: (item[1]-self.last_cx)**2 + (item[2]-self.last_cy)**2)
                    
#                     # 区分卡尔曼滤波和非卡尔曼滤波物体
#                     if self.locked_kind in ['brown', 'white', 'blue'] and self.locked_kind in kalman_coords:
#                         target_pos = kalman_coords[self.locked_kind]
#                         # 使用卡尔曼坐标更新last_cx, last_cy
#                         self.last_cx, self.last_cy = target_pos
#                     else:
#                         target_pos = (best_cx, best_cy)
#                         # 使用原始坐标更新last_cx, last_cy
#                         self.last_cx, self.last_cy = best_cx, best_cy
                    
#                     self.lost_count = 0
#                     locked_object = best_obj
#                 else:
#                     self.lost_count += 1
#         else:
#             if self.is_locked:
#                 self.lost_count += 1

#         # 锁定丢失逻辑
#         if self.is_locked and self.lost_count >= self.MAX_LOST_FRAMES:
#             self.reset()

#         target_kind = self.locked_kind if self.is_locked else ""
#         return target_pos, target_kind, locked_object

#     def draw_lock_mark(self, img, locked_object, kalman_coords):
#         """绘制锁定标识"""
#         if not self.is_locked:
#             return
        
#         lock_cx, lock_cy = None, None
        
#         # 对于卡尔曼滤波物体，优先使用卡尔曼坐标
#         if self.locked_kind in ['brown', 'white']:
#             if self.locked_kind in kalman_coords:
#                 kcx, kcy = kalman_coords[self.locked_kind]
#                 # 检查卡尔曼坐标是否有效
#                 if (kcx, kcy) != (SCREEN_CENTER_X, SCREEN_CENTER_Y):
#                     lock_cx, lock_cy = kcx, kcy
        
#         # 如果卡尔曼坐标不可用或非卡尔曼滤波物体，使用原始坐标
#         if lock_cx is None and locked_object is not None:
#             x1, y1, x2, y2, _, _ = locked_object
#             lock_cx = (x1 + x2) // 2
#             lock_cy = (y1 + y2) // 2
        
#         if lock_cx is not None and lock_cy is not None:
#             img.draw_circle(int(lock_cx), int(lock_cy), 5, color=DRAW_COLORS['black'], thickness=2)

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

# 存储各颜色卡尔曼坐标的字典
kalman_coords = {
    'brown': (SCREEN_CENTER_X, SCREEN_CENTER_Y),
    'white': (SCREEN_CENTER_X, SCREEN_CENTER_Y)
    # 'blue': (SCREEN_CENTER_X, SCREEN_CENTER_Y)
}

# 时间戳
last_time = time.ticks_ms()

# ======================== 工具函数 ========================
def handle_uart_commands(uart):
    """处理串口命令，切换运行模式"""
    global current_mode
    if uart.any():
        cmd = uart.read(1)

        def reset_all():
            brown_tracker.reset()
            white_tracker.reset()
            # target_locker.reset()

        if cmd == b'C':
            current_mode = MODE_CORRECTION
            reset_all()
        elif cmd == b'M':
            current_mode = MODE_MODEL
            reset_all()
        elif cmd == b'F':
            current_mode = MODE_WAITING
            reset_all()

# ======================== 初始化 ========================
# 检验是否成功运行程序并延时使其稳定
LED(4).on()
time.sleep_ms(500)
LED(4).off()

# 串口初始化
uart = UART(UART_PORT, baudrate=UART_BAUDRATE)
time.sleep_ms(100)  # 等待串口稳定

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(CAMERA_PIXFORMAT)
sensor.set_framesize(CAMERA_FRAMESIZE)
sensor.set_framerate(CAMERA_FRAMERATE)
sensor.set_auto_gain(False)  # 关闭自动增益
sensor.set_auto_whitebal(False)  # 关闭自动白平衡
sensor.set_brightness(CAMERA_BRIGHTNESS)
sensor.set_contrast(2) # 对比度
sensor.skip_frames(time=200)  # 跳过初始帧，让摄像头稳定
clock = time.clock()

# LCD初始化
lcd = seekfree.IPS200(3)
lcd.full()

# 创建模块实例
tag_corrector = CoordinateCorrection()
communicator = Communicator(uart)
brown_tracker = KalmanTracker()
white_tracker = KalmanTracker()
model_detector = ModelDetector(net)
# target_locker = TargetLocker(LOCK_JUMP_THRESHOLD, LOCK_MAX_LOST_FRAMES)

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
        center = [] # 本帧检测到的目标中心列表

        objects = model_detector.detect(img)
        brown_bear = [obj for obj in objects if LABEL_TO_COLOR.get(obj[4]) == 'brown' and obj[5] > 0.3]
        white_bear = [obj for obj in objects if LABEL_TO_COLOR.get(obj[4]) == 'white' and obj[5] > 0.3]
        other_objects = [(obj, LABEL_TO_COLOR.get(obj[4])) for obj in objects if LABEL_TO_COLOR.get(obj[4]) in ['red','blue','green'] and obj[5] > 0.6]

        model_detector.process_kalman_color(img, brown_bear, brown_tracker, 'brown', Ts, center, kalman_coords)
        model_detector.process_kalman_color(img, white_bear, white_tracker, 'white', Ts, center, kalman_coords)
        model_detector.draw_other_objects(img, other_objects, center)

        # target_pos, target_color, locked_blob = target_locker.process_lock(objects, kalman_coords)
        # target_locker.draw_lock_mark(img, locked_blob, kalman_coords)
        
        # # 发送目标坐标
        # if target_locker.is_locked and target_pos is not None:
        #     # 锁定状态：发送锁定目标坐标
        #     tx = int(round(target_pos[0]))
        #     ty = int(round(target_pos[1]))
        #     communicator.send_coordinate(tx, ty, target_locker.locked_kind)
        #     is_sent = True
        # elif not target_locker.is_locked and center:
        if center:
            target = max(center, key=lambda coordinate: coordinate[1])
            target_x = target[0]
            target_y = target[1]
            target_kind = target[2]
            communicator.send_coordinate(target_x, target_y, target_kind)
            global_counter += 1
            is_sent = True

        displayed_text = 'YES' if is_sent else 'NO'
        displayed_text_color = DRAW_COLORS['green'] if is_sent else DRAW_COLORS['red']
        img.draw_string(5, 5, displayed_text, color = displayed_text_color, scale = 2)

    # 显示图像到LCD
    lcd.show_image(img, SCREEN_WIDTH, SCREEN_HEIGHT, zoom=0)
    # print(clock.fps())