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

# 卡尔曼滤波配置
KALMAN_MAX_LOST_FRAMES = 3
MAX_SPEED = 80

# 锁定逻辑配置
LOCK_JUMP_THRESHOLD = 20  # 坐标跳变超过20像素视为同色干扰
LOCK_MAX_LOST_FRAMES = 3  # 丢失3帧解除锁定

# 通信协议常量
PROTOCOL_HEADER1 = 0xA5
PROTOCOL_HEADER2_COORD = 0xA6
PROTOCOL_HEADER2_ANGLE = 0xA7
PROTOCOL_HEADER2_APRILTAG = 0xA8
PROTOCOL_FOOTER = 0x5B

# 颜色类型ASCII码映射
COLOR_TYPE_MAP = {
    'red': ord('S'),
    'blue': ord('S'),
    'green': ord('T'),
    'brown': ord('B'),
    'white': ord('B'),
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

# 多套阈值
THRESHOLD = {'dark':{
    'brown':[(20, 66, -5, 23, 19, 66), # 27
    (14, 61, -7, 18, 12, 52)], # 30
    'red':[(15, 39, 18, 59, 17, 49), # 27
    (5, 34, 8, 52, -7, 41)], # 30
    'green':[(35, 84, -47, -14, -20, 87), # 27
    (32, 85, -43, -12, 13, 84)], # 30
    'blue':[(25, 53, -29, -9, -26, -8), # 27
    (17, 58, -24, -3, -36, -17), # 29
    (18, 53, -20, -5, -35, -13)], # 30
    'white':[(47, 76, -17, 0, 2, 39),  # 27
    (39, 79, -15, 2, -5, 9)] # 30
}, 'normal':{
    'brown':[(40, 83, -6, 22, 11, 69), # 45
    (31, 94, -5, 22, 8, 67)], # 50

    'red':[(32, 55, 38, 85, -23, 52), # 45
    (25, 56, 34, 87, 11, 53)], # 50
    'green':[(58, 100, -49, -20, 44, 97), # 45
    (71, 98, -52, -10, 39, 98)], # 50
    'blue':[(33, 79, -29, -2, -52, -24), # 45
    (33, 74, -31, -8, -46, -16), # 42
    (44, 83, -33, -6, -54, -27)], # 50
    'white':[(56, 100, -18, 0, -9, 16), # 45
    (69, 100, -23, 5, -5, 24)] # 50
}, 'bright':{
    'brown':[(40, 83, -6, 22, 11, 69)],
    'red':[(5, 24, 12, 41, -5, 37), (30, 58, 39, 83, 10, 51)],
    'green':[(17, 67, -33, -15, -15, 68), (53, 100, -51, -15, -20, 95)],
    'blue':[(13, 35, -24, -9, -18, -7), (37, 77, -31, -4, -54, -26)],
    'white':[(91, 100, -23, 4, -7, 20)] # 65
}
            }

# 亮度区间划分
BRIGHTNESS_RANGES = {
    'dark':(0, 35),
    'normal':(35, 58),
    'bright':(58, 100)
}

# 锁定的阈值集
LOCKED_THRESHOLD = None
LOCKED_TYPE = ''

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
        self.R = np.diag([4, 4, 10, 10, 50, 50])

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
            Q_value = [0.05, 0.05, 0.05, 0.05, 0.5, 0.5]
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
    
# ======================== 颜色检测模块 ========================
class ColorDetector:
    # 距离阈值（过滤过近的色块）
    DISTANCE_THRESHOLD = 400

    @staticmethod
    def calculate_distance(x1, y1, x2, y2):
        """计算两点间欧氏距离"""
        return (x1 - x2)**2 + (y1 - y2)**2

    def detect_colors(self, img):
        """检测所有颜色色块并返回（带颜色标签）"""
        current_threshold = LOCKED_THRESHOLD
        # 检测各颜色色块
        brown_blobs = img.find_blobs(current_threshold['brown'], pixels_threshold=200, area_threshold=200, merge=True)
        white_blobs = img.find_blobs(current_threshold['white'], pixels_threshold=200, area_threshold=200, merge=True)
        red_blobs   = img.find_blobs(current_threshold['red'],   pixels_threshold=80,  area_threshold=80,  merge=True)
        green_blobs = img.find_blobs(current_threshold['green'], pixels_threshold=75,  area_threshold=75,  merge=True)
        blue_blobs  = img.find_blobs(current_threshold['blue'],  pixels_threshold=110,  area_threshold=110,  merge=True)

        # 整合所有色块并添加颜色标签
        all_blobs = []
        for blob in brown_blobs: all_blobs.append((blob, 'brown'))
        for blob in white_blobs: all_blobs.append((blob, 'white'))
        for blob in red_blobs:   all_blobs.append((blob, 'red'))
        for blob in green_blobs: all_blobs.append((blob, 'green'))
        for blob in blue_blobs:  all_blobs.append((blob, 'blue'))
        return all_blobs

    def filter_all_blobs(self, blobs):
        """过滤无效色块（密度、像素数、长宽比、距离）"""
        filtered = []
        for blob, color in blobs:
            # 密度过滤（排除稀疏色块）
            if blob.density() < 0.4:
                continue
            elif color == 'green' and blob.density() < 0.55:
                continue

            # 长宽比过滤（不同颜色有不同规则）
            if color == 'brown' and (blob.w() > 3.5 * blob.h() or blob.h() > 3.5 * blob.w()):
                continue
            elif color == 'white' and (blob.w() > 3.5 * blob.h() or blob.h() > 3.5 * blob.w()):
                continue
            elif color == 'green' and (blob.w() > 1.3 * blob.h() or blob.h() > 1.3 * blob.w()):
                continue
            elif color == 'blue' and (blob.w() > 1.5 * blob.h() or blob.h() > 1.5 * blob.w()):
                continue

            # 距离过滤（排除与已保存色块过近的色块）
            cx, cy = blob.cx(), blob.cy()
            keep = True
            for saved_blob, _ in filtered:
                if self.calculate_distance(cx, cy, saved_blob.cx(), saved_blob.cy()) < self.DISTANCE_THRESHOLD:
                    keep = False
                    break
            if keep:
                filtered.append((blob, color))
        return filtered

    def process_kalman_color(self, img, blobs, tracker, color, Ts, center_list, kalman_coords_dict):
        """封装卡尔曼处理单颜色逻辑（棕/白/蓝通用）"""
        detected = False
        target_blob = None
        
        if blobs:
            # 选择面积最大的色块
            target_blob = max(blobs, key=lambda b: b.area())
            cx, cy = target_blob.cx(), target_blob.cy()
            w, h = target_blob.w(), target_blob.h()
            
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
            img.draw_rectangle(target_blob.rect(), color=DRAW_COLORS[color])
            img.draw_cross(cx, cy, color=DRAW_COLORS[color])

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
        
        return target_blob

    def draw_other_blobs(self, img, blobs, center_list):
        """封装绘制其他颜色色块逻辑"""
        for item in blobs:
            blob = item[0]
            color_name = item[1]
            # 绘制色块
            img.draw_rectangle(blob.rect(), color=DRAW_COLORS[color_name])
            img.draw_cross(blob.cx(), blob.cy(), color=DRAW_COLORS[color_name])
            # 添加到中心列表
            center_list.append((blob.cx(), blob.cy(), color_name))

# ======================== 锁定逻辑模块 ========================
class TargetLocker:
    def __init__(self, jump_threshold, max_lost_frames):
        self.is_locked = False        # 是否锁定目标
        self.locked_color = ''        # 锁定的目标颜色
        self.locked_cx = SCREEN_CENTER_X  # 锁定目标的初始x坐标
        self.locked_cy = SCREEN_CENTER_Y  # 锁定目标的初始y坐标
        self.last_cx = SCREEN_CENTER_X    # 上一帧锁定目标的x坐标
        self.last_cy = SCREEN_CENTER_Y    # 上一帧锁定目标的y坐标
        self.lost_count = 0           # 锁定目标丢失帧数
        self.JUMP_THRESHOLD = jump_threshold
        self.MAX_LOST_FRAMES = max_lost_frames

    def reset(self):
        """重置锁定状态"""
        self.is_locked = False
        self.locked_color = ''
        self.locked_cx = SCREEN_CENTER_X
        self.locked_cy = SCREEN_CENTER_Y
        self.last_cx = SCREEN_CENTER_X
        self.last_cy = SCREEN_CENTER_Y
        self.lost_count = 0

    def is_jump_too_large(self, cx, cy):
        """判断坐标跳变是否过大（同色干扰）"""
        squared_distance = (cx - self.last_cx)**2 + (cy - self.last_cy)**2
        return squared_distance > (self.JUMP_THRESHOLD**2)

    def process_lock(self, filtered_blobs, kalman_coords):
        """处理锁定逻辑，返回目标位置、目标颜色、锁定的色块"""
        target_pos = None
        target_color = ''
        locked_blob = None

        if filtered_blobs:
            # 未锁定：选择y最大的目标作为锁定对象
            if not self.is_locked:
                max_y_blob, max_y_color = max(
                    filtered_blobs,
                    key=lambda item: item[0].cy()
                )
                self.locked_color = max_y_color
                if self.locked_color in kalman_coords:
                    self.locked_cx, self.locked_cy = kalman_coords[self.locked_color]
                else:
                    self.locked_cx = max_y_blob.cx()
                    self.locked_cy = max_y_blob.cy()
                self.last_cx = self.locked_cx
                self.last_cy = self.locked_cy
                self.is_locked = True
                self.lost_count = 0
                target_pos = (self.locked_cx, self.locked_cy)
                target_color = max_y_color
                locked_blob = max_y_blob
            # 已锁定：仅筛选同色目标，且坐标跳变不超过阈值
            else:
                # 筛选同色目标
                same_color_blobs = [
                    item for item in filtered_blobs
                    if item[1] == self.locked_color
                ]
                valid_blobs = []
                for blob, color in same_color_blobs:
                    cx, cy = blob.cx(), blob.cy()
                    # 跳变不超过阈值才视为有效目标
                    if not self.is_jump_too_large(cx, cy):
                        valid_blobs.append((blob, cx, cy))

                if valid_blobs:
                    # 选同色目标中最接近上一帧锁定位置的
                    best_blob, best_cx, best_cy = min(
                        valid_blobs,
                        key=lambda item:(item[1]-self.last_cx)**2 + (item[2]-self.last_cy)**2
                    )
                    if self.locked_color in kalman_coords:
                        target_pos = kalman_coords[self.locked_color]
                    else:
                        target_pos = (best_cx, best_cy)
                    self.last_cx = best_cx
                    self.last_cy = best_cy
                    self.lost_count = 0
                    target_color = self.locked_color
                    locked_blob = best_blob
                else:
                    # 无有效同色目标，计数+1
                    self.lost_count += 1
                    if self.locked_color in kalman_coords and self.is_locked:
                        target_pos = kalman_coords[self.locked_color]
                    else:
                        target_pos = None
        else:
            # 无任何色块，锁定计数+1
            if self.is_locked:
                self.lost_count += 1
                if self.locked_color in kalman_coords:
                    target_pos = kalman_coords[self.locked_color]
                else:
                    target_pos = None
            else:
                target_pos = None

        # 超过最大丢失帧数，解除锁定
        if self.is_locked and self.lost_count >= self.MAX_LOST_FRAMES:
            self.reset()
            target_color = ''

        return target_pos, target_color, locked_blob

    def draw_lock_mark(self, img, locked_blob, kalman_coords):
        """绘制锁定标识（黑色圆）"""
        if self.is_locked and locked_blob is not None:
            if self.locked_color in kalman_coords:
                lock_cx, lock_cy = kalman_coords[self.locked_color]
            else:
                lock_cx = locked_blob.cx()
                lock_cy = locked_blob.cy()
            img.draw_circle(lock_cx, lock_cy, 5, color=DRAW_COLORS['black'], thickness=2)
    
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
MODE_TARGET = 0          # 目标跟踪模式
MODE_CORRECTION = 1      # 坐标校正
MODE_WAITING = 2         # 等待模式
current_mode = MODE_WAITING

# 存储各颜色卡尔曼坐标的字典
kalman_coords = {
    'brown': (SCREEN_CENTER_X, SCREEN_CENTER_Y),
    'white': (SCREEN_CENTER_X, SCREEN_CENTER_Y),
    'blue': (SCREEN_CENTER_X, SCREEN_CENTER_Y)
}

# 时间戳
last_time = time.ticks_ms()

# ======================== 工具函数 ========================
def handle_uart_commands(uart):
    """处理串口命令，切换运行模式"""
    global current_mode
    if uart.any():
        cmd = uart.read(1)
        if cmd == b'T':
            current_mode = MODE_TARGET
            brown_tracker.reset()
            white_tracker.reset()
            blue_tracker.reset()
            target_locker.reset()
        elif cmd == b'C':
            current_mode = MODE_CORRECTION
            brown_tracker.reset()
            white_tracker.reset()
            blue_tracker.reset()
            target_locker.reset()
        elif cmd == b'F':
            current_mode = MODE_WAITING
            brown_tracker.reset()
            white_tracker.reset()
            blue_tracker.reset()
            target_locker.reset()

def init_threshold():
    """依据当前亮度自动匹配阈值"""
    global LOCKED_THRESHOLD, LOCKED_TYPE
    img = sensor.snapshot()

    stats = img.get_statistics()
    l_mean = stats.l_mean()

    for type, (min_brightness, max_brightness) in BRIGHTNESS_RANGES.items():
        if min_brightness <= l_mean <= max_brightness:
            LOCKED_TYPE = type
            LOCKED_THRESHOLD = THRESHOLD[LOCKED_TYPE]
            return

    LOCKED_TYPE = 'normal'
    LOCKED_THRESHOLD = THRESHOLD['normal']
    return

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
sensor.skip_frames(time=200)  # 跳过初始帧，让摄像头稳定
clock = time.clock()

init_threshold()

# LCD初始化
lcd = seekfree.IPS200(3)
lcd.full()

# 创建模块实例
color_detector = ColorDetector()
tag_corrector = CoordinateCorrection()
brown_tracker = KalmanTracker()
white_tracker = KalmanTracker()
blue_tracker = KalmanTracker()
communicator = Communicator(uart)
# 初始化锁定类实例
target_locker = TargetLocker(LOCK_JUMP_THRESHOLD, LOCK_MAX_LOST_FRAMES)

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

    # 目标跟踪模式
    elif current_mode == MODE_TARGET:
        # 色块检测与筛选
        all_blobs_with_color = color_detector.detect_colors(img)
        filtered_blobs_with_color = color_detector.filter_all_blobs(all_blobs_with_color)

        # 初始化变量
        center = []  # 所有有效色块的中心坐标 (cx, cy, color)
        target_pos = None  # 最终要发送的目标坐标
        locked_blob = None  # 锁定的目标色块
        target_color = ''
        is_sent = False # 是否发送了坐标

        # 分离棕色、白色、蓝色与其它色块
        brown_blobs = []
        white_blobs = []
        blue_blobs = []
        other_blobs = []
        for item in filtered_blobs_with_color:
            blob = item[0]
            # print(blob.density(),blob.pixels())
            color = item[1]
            if color == 'brown':
                brown_blobs.append(blob)
            elif color == 'white':
                white_blobs.append(blob)
            elif color == 'blue':
                blue_blobs.append(blob)
            else:
                other_blobs.append((blob, color))

        color_detector.process_kalman_color(img, brown_blobs, brown_tracker, 'brown', Ts, center, kalman_coords)
        color_detector.process_kalman_color(img, white_blobs, white_tracker, 'white', Ts, center, kalman_coords)
        color_detector.process_kalman_color(img, blue_blobs, blue_tracker, 'blue', Ts, center, kalman_coords)
        color_detector.draw_other_blobs(img, other_blobs, center)

        target_pos, target_color, locked_blob = target_locker.process_lock(filtered_blobs_with_color, kalman_coords)
        target_locker.draw_lock_mark(img, locked_blob, kalman_coords)

        # 发送目标坐标
        if target_locker.is_locked and target_pos is not None:
            # 锁定状态：发送锁定目标坐标
            communicator.send_coordinate(target_pos[0], target_pos[1], target_locker.locked_color)
            is_sent = True
        elif not target_locker.is_locked and center:
            # 未锁定：按原有逻辑选y最大的坐标
            target = max(center, key=lambda coordinate: coordinate[1])
            target_x = target[0]
            target_y = target[1]
            target_color = target[2]
            communicator.send_coordinate(target_x, target_y, target_color)
            is_sent = True
        # 锁定状态但没识别到目标和未锁定状态但未检测到色块，均不发送坐标
        displayed_text = 'YES' if is_sent else 'NO'
        displayed_text_color = DRAW_COLORS['green'] if is_sent else DRAW_COLORS['red']
        img.draw_string(5, 5, displayed_text, color = displayed_text_color, scale = 2)

    # 坐标校正模式
    elif current_mode == MODE_CORRECTION:
        tag_center = tag_corrector.coordinate_correction(img)
        if tag_center is not None:
            tag_cx, tag_cy, rotation = tag_center
            rotation = (180 * rotation) / math.pi + 90
            communicator.send_coordinate_with_angle(tag_cx, tag_cy, rotation)

    # 显示图像到LCD
    lcd.show_image(img, SCREEN_WIDTH, SCREEN_HEIGHT, zoom=0)
    # (clock.fps())