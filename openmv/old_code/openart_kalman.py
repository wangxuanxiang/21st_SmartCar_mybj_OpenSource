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

# ======================== 常量定义（集中管理，便于修改） ========================
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
KALMAN_INIT_POS_X = SCREEN_CENTER_X
KALMAN_INIT_POS_Y = SCREEN_CENTER_Y
MAX_SPEED = 80

# 锁定逻辑配置
LOCK_JUMP_THRESHOLD = 20  # 坐标跳变超过20像素视为同色干扰
LOCK_MAX_LOST_FRAMES = 3  # 丢失5帧解除锁定

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
BRIGHTNESS_RANGES ={
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
        # 坐标取整
        x = int(round(x))
        y = int(round(y))

        # 防抖：变化量过小且y坐标在上方时不发送
        if abs(x - self.last_sent_x) < 3 and abs(y - self.last_sent_y) < 3 and y <= 40:
            return

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

# ======================== 颜色检测模块 ========================
class ColorDetector:

    """
    # 颜色阈值（类变量，共享）
    RED_THRESHOLD = [(5, 24, 12, 41, -5, 37), (30, 58, 39, 83, 10, 51)]
    GREEN_THRESHOLD = [(17, 67, -33, -15, -15, 68), (53, 100, -51, -15, -20, 95)]
    BLUE_THRESHOLD = [(13, 35, -24, -9, -18, -7), (37, 77, -31, -4, -54, -26)]
    BROWN_THRESHOLD = [(12, 43, -14, 14, 8, 46), (51, 92, -23, 20, -16, 70)]
    WHITE_THRESHOLD = []
    """
    # 定义中心采样区 (x, y, w, h) - 针对 160x120 图像，取中心 40x30 区域
    CENTER_ROI = (60, 45, 40, 30)

    # 距离阈值（过滤过近的色块）
    DISTANCE_THRESHOLD = 30

    @staticmethod
    def calculate_distance(x1, y1, x2, y2):
        """计算两点间欧氏距离"""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

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
            """
            # 像素数过滤（动态最小像素数）
            min_pixels = 30 * (blob.density() + 0.5)
            if blob.pixels() < min_pixels:
                continue
            """
            """
            if (blob.w() > 140 or blob.h() > 110):
                continue
            """

            # 长宽比过滤（不同颜色有不同规则）
            if color == 'brown' and (blob.w() > 3.5 * blob.h() or blob.h() > 3.5 * blob.w()):
                continue
            if color == 'white' and (blob.w() > 3.5 * blob.h() or blob.h() > 3.5 * blob.w()):
                continue
            if color in ('green', 'blue') and (blob.w() > 1.5 * blob.h() or blob.h() > 1.5 * blob.w()):
                continue

            # 距离过滤（排除与已保存色块过近的色块）
            cx, cy = blob.cx(), blob.cy()
            keep = True
            for saved_blob, _ in filtered:
                d = self.calculate_distance(cx, cy, saved_blob.cx(), saved_blob.cy())
                if d < self.DISTANCE_THRESHOLD:
                    keep = False
                    break
            if keep:
                filtered.append((blob, color))
        return filtered

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
        self.last_brown_area = 0
        self.last_cx, self.last_cy = 80, 60  # 初始中心坐标
        # 初始状态x_hat：[x, y, w, h, vx, vy]
        self.x_hat = np.array([80, 60, 30, 30, 2, 2], dtype=np.float)
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
            S_inv = np.linalg.inv(S_reg)
            K = np.dot(np.dot(p_minus, self.C.T), S_inv)
            self.x_hat = x_hat_minus + np.dot(K, (Z - np.dot(self.C, x_hat_minus)))
            self.p = np.dot((np.eye(6) - np.dot(K, self.C)), p_minus)
        else:
            self.x_hat = x_hat_minus
            self.p = p_minus

        return self.x_hat

# ======================== 全局状态变量 ========================
# 模式定义
MODE_TARGET = 0          # 目标跟踪模式
MODE_CORRECTION = 1      # 坐标校正
MODE_WAITING = 2         # 等待模式
current_mode = MODE_WAITING
# 锁定状态变量
is_target_locked = False        # 是否锁定目标
locked_target_color = ''        # 锁定的目标颜色
locked_target_cx = SCREEN_CENTER_X  # 锁定目标的初始x坐标
locked_target_cy = SCREEN_CENTER_Y  # 锁定目标的初始y坐标
locked_last_cx = SCREEN_CENTER_X    # 上一帧锁定目标的x坐标
locked_last_cy = SCREEN_CENTER_Y    # 上一帧锁定目标的y坐标
locked_lost_count = 0           # 锁定目标丢失帧数

# 新增：存储各颜色卡尔曼坐标的字典
kalman_coords = {
    'brown': (SCREEN_CENTER_X, SCREEN_CENTER_Y),
    'white': (SCREEN_CENTER_X, SCREEN_CENTER_Y),
    'blue': (SCREEN_CENTER_X, SCREEN_CENTER_Y)
}

# 时间戳
last_time = time.ticks_ms()

# ======================== 工具函数 ========================
def reset_lock_state():
    """重置锁定状态"""
    global is_target_locked, locked_target_color, locked_target_cx, locked_target_cy
    global locked_last_cx, locked_last_cy, locked_lost_count
    is_target_locked = False
    locked_target_color = ''
    locked_target_cx = SCREEN_CENTER_X
    locked_target_cy = SCREEN_CENTER_Y
    locked_last_cx = SCREEN_CENTER_X
    locked_last_cy = SCREEN_CENTER_Y
    locked_lost_count = 0

def is_coordinate_jump_too_large(cx, cy):
    """判断坐标跳变是否过大（同色干扰）"""
    global locked_last_cx, locked_last_cy
    distance = math.sqrt((cx - locked_last_cx)**2 + (cy - locked_last_cy)**2)
    return distance > LOCK_JUMP_THRESHOLD

def handle_uart_commands():
    """处理串口命令，切换运行模式"""
    global current_mode
    if uart.any():
        cmd = uart.read(1)
        if cmd == b'T':
            current_mode = MODE_TARGET
        elif cmd == b'C':
            current_mode = MODE_CORRECTION
        elif cmd == b'F':
            current_mode = MODE_WAITING

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
# 新增：蓝色卡尔曼跟踪实例
blue_tracker = KalmanTracker()
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
    handle_uart_commands()

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

        # 分离棕色、白色、蓝色与其它色块
        brown_blobs = []
        white_blobs = []
        blue_blobs = []
        other_blobs = []
        for item in filtered_blobs_with_color:
            blob = item[0]
            color = item[1]
            if color == 'brown':
                brown_blobs.append(blob)
            elif color == 'white':
                white_blobs.append(blob)
            elif color == 'blue':
                blue_blobs.append(blob)
            else:
                other_blobs.append((blob, color))

        # 处理棕色色块
        brown_detected = False
        if brown_blobs:
            # 选择面积最大的棕色色块
            target_brown = max(brown_blobs, key=lambda b: b.area())
            cx, cy = target_brown.cx(), target_brown.cy()
            w, h = target_brown.w(), target_brown.h()
            brown_pos = (target_brown.cx(), target_brown.cy())
            dx_raw = (cx - brown_tracker.last_cx) / Ts
            dx = max(-MAX_SPEED, min(MAX_SPEED, dx_raw))
            dy_raw = (cy - brown_tracker.last_cy) / Ts
            dy = max(-MAX_SPEED, min(MAX_SPEED, dy_raw))

            # 构建6维新观测值Z
            Z = np.array([cx, cy, w, h, dx, dy], dtype = np.float)

            # 第一次检测到目标时初始化卡尔曼
            if not brown_tracker.first_detected:
                brown_tracker.x_hat = np.array([cx, cy, w, h, 0, 0], dtype=np.float)
                brown_tracker.p = np.diag([10.0, 10.0, 5.0, 5.0, 100.0, 100.0])
                brown_tracker.first_detected = True

            # 卡尔曼滤波更新
            brown_tracker.kalman_filter(Z, Ts, True)
            brown_tracker.last_cx = cx
            brown_tracker.last_cy = cy
            brown_detected = True

            # 绘制原始检测框
            img.draw_rectangle(target_brown.rect(), color=DRAW_COLORS['brown'])
            img.draw_cross(cx, cy, color=DRAW_COLORS['brown'])

        # 无检测时卡尔曼预测（棕色）
        if not brown_detected and brown_tracker.first_detected:
            if brown_tracker.lost_count < brown_tracker.MAX_LOST_FRAMES:
                brown_tracker.kalman_filter(None, Ts, is_detected=False)
            else:
                brown_tracker.reset()

        # 绘制卡尔曼预测框（棕色-灰色）
        if brown_tracker.first_detected:
            kcx, kcy = int(brown_tracker.x_hat[0]), int(brown_tracker.x_hat[1])
            kw, kh = max(1, int(brown_tracker.x_hat[2])), max(1, int(brown_tracker.x_hat[3]))
            img.draw_rectangle(kcx - kw//2, kcy - kh//2, kw, kh, color=DRAW_COLORS['grey'])
            img.draw_cross(kcx, kcy, color=DRAW_COLORS['grey'])
            center.append((kcx, kcy, 'brown'))
            kalman_coords['brown'] = (kcx, kcy)
        else:
            kalman_coords['brown'] = (SCREEN_CENTER_X, SCREEN_CENTER_Y)
        # 处理白色色块
        white_detected = False
        if white_blobs:
            target_white = max(white_blobs, key = lambda b:b.area())
            cx, cy = target_white.cx(), target_white.cy()
            w, h = target_white.w(), target_white.h()

            # 计算速度
            dx_raw = (cx - white_tracker.last_cx) / Ts
            dx = max(-MAX_SPEED, min(MAX_SPEED, dx_raw))
            dy_raw = (cy - white_tracker.last_cy) / Ts
            dy = max(-MAX_SPEED, min(MAX_SPEED, dy_raw))

            # 构建6维新观测值Z
            Z = np.array([cx, cy, w, h, dx, dy], dtype = np.float)

            # 第一次检测到目标时初始化卡尔曼
            if not white_tracker.first_detected:
                white_tracker.x_hat = np.array([cx, cy, w, h, 0, 0], dtype=np.float)
                white_tracker.p = np.diag([10.0, 10.0, 5.0, 5.0, 100.0, 100.0])
                white_tracker.first_detected = True

            # 卡尔曼滤波更新
            white_tracker.kalman_filter(Z, Ts, True)
            white_tracker.last_cx = cx
            white_tracker.last_cy = cy
            white_detected = True

            # 绘制原始检测框
            img.draw_rectangle(target_white.rect(), color=DRAW_COLORS['white'])
            img.draw_cross(cx, cy, color=DRAW_COLORS['white'])

        # 无检测时卡尔曼预测（白色）
        if not white_detected and white_tracker.first_detected:
            if white_tracker.lost_count < white_tracker.MAX_LOST_FRAMES:
                white_tracker.kalman_filter(None, Ts, is_detected=False)
            else:
                white_tracker.reset()

        # 绘制卡尔曼预测框（白色-灰色）
        if white_tracker.first_detected:
            kcx, kcy = int(white_tracker.x_hat[0]), int(white_tracker.x_hat[1])
            kw, kh = max(1, int(white_tracker.x_hat[2])), max(1, int(white_tracker.x_hat[3]))
            img.draw_rectangle(kcx - kw//2, kcy - kh//2, kw, kh, color=DRAW_COLORS['grey'])
            img.draw_cross(kcx, kcy, color=DRAW_COLORS['grey'])
            center.append((kcx, kcy, 'white'))
            kalman_coords['white'] = (kcx, kcy)
        else:
            kalman_coords['white'] = (SCREEN_CENTER_X, SCREEN_CENTER_Y)

        # 处理蓝色色块
        blue_detected = False
        if blue_blobs:
            # 选择面积最大的蓝色色块
            target_blue = max(blue_blobs, key = lambda b:b.area())
            cx, cy = target_blue.cx(), target_blue.cy()
            w, h = target_blue.w(), target_blue.h()

            # 计算速度
            dx_raw = (cx - blue_tracker.last_cx) / Ts
            dx = max(-MAX_SPEED, min(MAX_SPEED, dx_raw))
            dy_raw = (cy - blue_tracker.last_cy) / Ts
            dy = max(-MAX_SPEED, min(MAX_SPEED, dy_raw))

            # 构建6维新观测值Z
            Z = np.array([cx, cy, w, h, dx, dy], dtype = np.float)

            # 第一次检测到目标时初始化卡尔曼
            if not blue_tracker.first_detected:
                blue_tracker.x_hat = np.array([cx, cy, w, h, 0, 0], dtype=np.float)
                blue_tracker.p = np.diag([10.0, 10.0, 5.0, 5.0, 100.0, 100.0])
                blue_tracker.first_detected = True

            # 卡尔曼滤波更新
            blue_tracker.kalman_filter(Z, Ts, True)
            blue_tracker.last_cx = cx
            blue_tracker.last_cy = cy
            blue_detected = True

            # 绘制原始检测框（蓝色）
            img.draw_rectangle(target_blue.rect(), color=DRAW_COLORS['blue'])
            img.draw_cross(cx, cy, color=DRAW_COLORS['blue'])

        # 无检测时卡尔曼预测（蓝色）
        if not blue_detected and blue_tracker.first_detected:
            if blue_tracker.lost_count < blue_tracker.MAX_LOST_FRAMES:
                blue_tracker.kalman_filter(None, Ts, is_detected=False)
            else:
                blue_tracker.reset()

        # 绘制卡尔曼预测框（蓝色-灰色）
        if blue_tracker.first_detected:
            kcx, kcy = int(blue_tracker.x_hat[0]), int(blue_tracker.x_hat[1])
            kw, kh = max(1, int(blue_tracker.x_hat[2])), max(1, int(blue_tracker.x_hat[3]))
            img.draw_rectangle(kcx - kw//2, kcy - kh//2, kw, kh, color=DRAW_COLORS['grey'])
            img.draw_cross(kcx, kcy, color=DRAW_COLORS['grey'])
            center.append((kcx, kcy, 'blue'))
            kalman_coords['blue'] = (kcx, kcy)
        else:
            kalman_coords['blue'] = (SCREEN_CENTER_X, SCREEN_CENTER_Y)

        # 处理其他颜色色块
        for item in other_blobs:
            blob = item[0]
            color_name = item[1]
            # 绘制色块
            img.draw_rectangle(blob.rect(), color=DRAW_COLORS[color_name])
            img.draw_cross(blob.cx(), blob.cy(), color=DRAW_COLORS[color_name])
            # 添加到中心列表
            center.append((blob.cx(), blob.cy(), color_name))

        # 锁定处理逻辑
        if filtered_blobs_with_color:
            # 未锁定：选择y最大的目标作为锁定对象
            if not is_target_locked:
                max_y_blob, max_y_color = max(
                    filtered_blobs_with_color,
                    key=lambda item: item[0].cy()
                )
                locked_target_color = max_y_color
                if locked_target_color in kalman_coords:
                    locked_target_cx, locked_target_cy = kalman_coords[locked_target_color]
                else:
                    locked_target_cx = max_y_blob.cx()
                    locked_target_cy = max_y_blob.cy()
                locked_last_cx = locked_target_cx
                locked_last_cy = locked_target_cy
                is_target_locked = True
                locked_lost_count = 0
                target_pos = (locked_target_cx, locked_target_cy)
                target_color = max_y_color
                locked_blob = max_y_blob
            # 已锁定：仅筛选同色目标，且坐标跳变不超过阈值
            else:
                # 筛选同色目标
                same_color_blobs = [
                    item for item in filtered_blobs_with_color
                    if item[1] == locked_target_color
                ]
                valid_blobs = []
                for blob, color in same_color_blobs:
                    cx, cy = blob.cx(), blob.cy()
                    # 跳变不超过阈值才视为有效目标
                    if not is_coordinate_jump_too_large(cx, cy):
                        valid_blobs.append((blob, cx, cy))

                if valid_blobs:
                    # 选同色目标中最接近上一帧锁定位置的
                    best_blob, best_cx, best_cy = min(
                        valid_blobs,
                        key=lambda item: math.sqrt(
                            (item[1]-locked_last_cx)**2 + (item[2]-locked_last_cy)**2
                        )
                    )
                    if locked_target_color in kalman_coords:
                        target_pos = kalman_coords[locked_target_color]
                    else:
                        target_pos = (best_cx, best_cy)
                    locked_last_cx = best_cx
                    locked_last_cy = best_cy
                    locked_lost_count = 0
                    target_color = locked_target_color
                    locked_blob = best_blob
                else:
                    # 无有效同色目标，计数+1
                    locked_lost_count += 1
                    if locked_target_color in kalman_coords and is_target_locked:
                        target_pos = kalman_coords[locked_target_color]
                    else:
                        target_pos = None
        else:
            # 无任何色块，锁定计数+1
            if is_target_locked:
                locked_lost_count += 1
                if locked_target_color in kalman_coords:
                    target_pos = kalman_coords[locked_target_color]
                else:
                    target_pos = None
            else:
                target_pos = None
            

        # 超过最大丢失帧数，解除锁定
        if is_target_locked and locked_lost_count >= LOCK_MAX_LOST_FRAMES:
            reset_lock_state()
            target_color = ''

        # 绘制锁定标识（黑色圆）
        if is_target_locked and locked_blob is not None:
            if locked_target_color in kalman_coords:
                lock_cx, lock_cy = kalman_coords[locked_target_color]
            else:
                lock_cx = locked_blob.cx()
                lock_cy = locked_blob.cy()
            img.draw_circle(lock_cx, lock_cy, 5, color=DRAW_COLORS['black'], thickness=2)

        # 发送目标坐标
        if is_target_locked and target_pos is not None:
            # 锁定状态：发送锁定目标坐标
            communicator.send_coordinate(target_pos[0], target_pos[1], locked_target_color)
        elif center:
            # 未锁定：按原有逻辑选y最大的坐标
            target = max(center, key=lambda coordinate: coordinate[1])
            target_x = target[0]
            target_y = target[1]
            target_color = target[2]
            communicator.send_coordinate(target_x, target_y, target_color)

    # 坐标校正模式
    elif current_mode == MODE_CORRECTION:
        tag_center = tag_corrector.coordinate_correction(img)
        if tag_center is not None:
            tag_cx, tag_cy, rotation = tag_center
            rotation = (180 * rotation) / math.pi + 90
            communicator.send_coordinate_with_angle(tag_cx, tag_cy, rotation)

    # 显示图像到LCD
    lcd.show_image(img, SCREEN_WIDTH, SCREEN_HEIGHT, zoom=0)
    print(clock.fps())