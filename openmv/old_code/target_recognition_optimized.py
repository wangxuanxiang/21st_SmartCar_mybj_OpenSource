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
CAMERA_BRIGHTNESS = 1000

# 屏幕尺寸
SCREEN_WIDTH = 160
SCREEN_HEIGHT = 120
SCREEN_CENTER_X = SCREEN_WIDTH // 2  # 80
SCREEN_CENTER_Y = SCREEN_HEIGHT // 2  # 60

# 卡尔曼滤波配置
KALMAN_MAX_LOST_FRAMES = 20
KALMAN_INIT_POS_X = SCREEN_CENTER_X
KALMAN_INIT_POS_Y = SCREEN_CENTER_Y

# 锁定逻辑配置
LOCK_JUMP_THRESHOLD = 20  # 坐标跳变超过20像素视为同色干扰
LOCK_MAX_LOST_FRAMES = 5  # 丢失5帧解除锁定

# 通信协议常量
PROTOCOL_HEADER1 = 0xA5
PROTOCOL_HEADER2_COORD = 0xA6
PROTOCOL_HEADER2_ANGLE = 0xA7
PROTOCOL_FOOTER = 0x5B

# 颜色类型ASCII码映射
COLOR_TYPE_MAP = {
    'red': ord('S'),
    'blue': ord('S'),
    'green': ord('T'),
    'brown': ord('B'),
    'white': ord('W'),
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

    def send_angle(self, angle):
        """发送边界矫正角度（映射到0~180）"""
        if angle is None:
            return
        # 角度映射并限制范围
        angle_mapped = angle + 90  # 映射到 0～180
        angle_mapped = max(0, min(180, angle_mapped))  # 范围限制
        angle_mapped = int(round(angle_mapped))        # 取整

        # 打包并发送数据
        data = ustruct.pack(
            "<BBBB",
            PROTOCOL_HEADER1,
            PROTOCOL_HEADER2_ANGLE,
            angle_mapped,
            PROTOCOL_FOOTER
        )
        self.uart.write(data)

# ======================== 颜色检测模块 ========================
class ColorDetector:
    # 颜色阈值（类变量，共享）
    RED_THRESHOLD = [(5, 24, 12, 41, -5, 37), (30, 58, 39, 83, 10, 51)]
    GREEN_THRESHOLD = [(17, 67, -33, -15, -15, 68), (53, 100, -51, -15, -20, 95)]
    BLUE_THRESHOLD = [(13, 35, -24, -9, -18, -7), (37, 77, -31, -4, -54, -26)]
    BROWN_THRESHOLD = [(12, 43, -14, 14, 8, 46), (51, 92, -23, 20, -16, 70)]
    WHITE_THRESHOLD = []

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
        """
        adjusted_brown = [self.auto_adjust_threshold(img, th) for th in self.BROWN_THRESHOLD]
        adjusted_red = [self.auto_adjust_threshold(img, th) for th in self.RED_THRESHOLD]
        adjusted_green = [self.auto_adjust_threshold(img, th) for th in self.GREEN_THRESHOLD]
        adjusted_blue = [self.auto_adjust_threshold(img, th) for th in self.BLUE_THRESHOLD]
        """
        # 检测各颜色色块
        brown_blobs = img.find_blobs(self.BROWN_THRESHOLD, pixels_threshold=400, area_threshold=400, merge=True)
        white_blobs = img.find_blobs(self.WHITE_THRESHOLD, pixels_threshold=400, area_threshold=400, merge=True)
        red_blobs   = img.find_blobs(self.RED_THRESHOLD,   pixels_threshold=30,  area_threshold=30,  merge=False)
        green_blobs = img.find_blobs(self.GREEN_THRESHOLD, pixels_threshold=30,  area_threshold=30,  merge=False)
        blue_blobs  = img.find_blobs(self.BLUE_THRESHOLD,  pixels_threshold=30,  area_threshold=30,  merge=False)
        """
        brown_blobs = img.find_blobs(adjusted_brown, pixels_threshold=200, area_threshold=200, merge=True)
        red_blobs   = img.find_blobs(adjusted_red,   pixels_threshold=30,  area_threshold=30,  merge=False)
        green_blobs = img.find_blobs(adjusted_green, pixels_threshold=30,  area_threshold=30,  merge=False)
        blue_blobs  = img.find_blobs(adjusted_blue,  pixels_threshold=30,  area_threshold=30,  merge=False)
        """
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
            if blob.density() < 0.3:
                continue

            # 像素数过滤（动态最小像素数）
            min_pixels = 50 * (blob.density() + 0.5)
            if blob.pixels() < min_pixels:
                continue

            """
            if (blob.w() > 140 or blob.h() > 110):
                continue
            """

            # 长宽比过滤（不同颜色有不同规则）
            if color == 'brown' and (blob.w() > 3 * blob.h() or blob.h() > 3 * blob.w()):
                continue
            if color == 'white' and (blob.w() > 3 * blob.h() or blob.h() > 3 * blob.w()):
                continue
            if color in ('green', 'blue') and (blob.w() > 1.5 * blob.h() or blob.h() > 1.5 * blob.w() or abs(blob.w() - blob.h()) > 10):
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

    def auto_adjust_threshold(self, img, base_threshold):
        """根据中心区域亮度自动调整颜色阈值"""
        stats = img.get_statistics(roi = self.CENTER_ROI)
        l_mean = stats.l_mean()

        target_brightness = 50  # 目标亮度
        brightness_diff = l_mean - target_brightness
        adjust_factor = 0.3  # 0.1-0.5之间调整，越小越平滑
        diff = brightness_diff * adjust_factor
        dead_zone = 2  # 亮度在48-52之间时，不调整阈值
        if abs(brightness_diff) < dead_zone:
            diff = 0

        # 调整亮度通道阈值
        l_low = base_threshold[0] + diff
        l_high = base_threshold[1] + diff
        l_low = max(0, min(100, l_low))
        l_high = max(0, min(100, l_high))

        # 保证l_low < l_high（避免阈值交叉导致曝光异常）
        if l_low >= l_high:
            l_low = max(0, l_high - 5)  # 至少保留5的阈值差

        # 拼接调整后的阈值
        threshold_part = base_threshold[2:]
        return (round(l_low), round(l_high)) + threshold_part  # 取整适配OpenART

# ======================== 边界检测模块 ========================
class BoundaryDetector:
    # 黄色边界阈值
    YELLOW_THRESHOLD = [(58, 98, -46, -17, 31, 101), (24, 51, -25, -4, 22, 55)]
    # 回归分析ROI（中间区域）
    ROI_MID = (40, 0, 80, 120)
    # X轴容差（中心偏移允许范围）
    X_TOLERANCE = 5

    def boundary_correction(self, mode, img):
        """
        边界矫正角度计算
        mode: 'row'（行/左右）、'column'（列/上下）
        return: 矫正角度（None表示检测失败）
        """
        angle = None
        blobs = []
        center_count = 0  # 重命名：center -> center_count，避免歧义

        # 分区域检测黄色色块
        if mode == 'row':  # 行：从左到右检测
            roi_x_list = [0, 26, 52, 80, 106, 132]
        elif mode == 'column':  # 列：从上到下检测
            roi_x_list = [0, 20, 40, 60, 80, 100]
        else:
            return None

        for x in roi_x_list:
            if mode == 'row':
                # 行检测ROI：[x, 0, 26, 120]
                roi = [x, 0, 26, SCREEN_HEIGHT]
            else:
                # 列检测ROI：[0, x, 160, 20]
                roi = [0, x, SCREEN_WIDTH, 20]

            # 检测黄色色块
            result = img.find_blobs(
                self.YELLOW_THRESHOLD,
                roi=roi,
                pixels_threshold=100,
                area_threshold=100,
                margin=1,
                merge=True,
                invert=0
            )

            if result:
                # 选择面积最接近600的色块
                best_blob = min(result, key=lambda b: abs(b.area() - 600))  # 面积还要调整
                blobs.append(best_blob)
                center_count += 1
                img.draw_rectangle(best_blob.rect(), color=(255, 0, 0), scale=1, thickness=1)

        # 至少检测到3个色块才进行回归分析
        if center_count >= 3:
            l = img.get_regression(self.YELLOW_THRESHOLD, roi=self.ROI_MID, robust=True)
            if l:
                img.draw_line(l.line(), color=(255, 0, 0), thickness=2)
                x1, y1, x2, y2 = l.line()

                # 确定底部x坐标
                bottom_x = x1 if y1 > y2 else x2

                # 中心偏移在容差范围内才计算角度
                if abs(bottom_x - SCREEN_CENTER_X) <= self.X_TOLERANCE:
                    theta = l.theta()
                    angle = theta - 180 if theta > 90 else theta
                return angle
            else:
                return None
        else:
            return None

# ======================== 卡尔曼跟踪模块 ========================
class KalmanTracker:
    MAX_LOST_FRAMES = KALMAN_MAX_LOST_FRAMES

    def __init__(self):
        # 1. 状态量定义: [x, y, vx, vy]
        self.x_hat = np.array([KALMAN_INIT_POS_X, KALMAN_INIT_POS_Y, 0, 0], dtype=np.float)

        # 2. 预分配矩阵 (内存优化：避免循环内创建对象)
        self.A = np.eye(4)      # 状态转移矩阵
        self.P = np.eye(4) * 10 # 后验协方差
        self.Q = np.eye(4) * 0.5# 过程噪声
        self.R = np.eye(2) * 5  # 测量噪声 (x, y)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float) # 观测矩阵

        self.first_detected = False
        self.lost_count = 0
        self.last_w = 0
        self.last_h = 0

    def reset(self):
        """重置跟踪器状态"""
        self.first_detected = False
        self.lost_count = 0
        self.x_hat = np.array([KALMAN_INIT_POS_X, KALMAN_INIT_POS_Y, 0, 0], dtype=np.float)
        self.P = np.eye(4) * 10
        self.last_w = 0
        self.last_h = 0

    """
    def is_valid(self, blob):
        current_area = blob.area()
        if self.last_brown_area > 0:
            rate = abs(current_area - self.last_brown_area) / self.last_brown_area
            if rate > 0.7:
                return False
        self.last_brown_area = current_area
        self.brown_visible_frames += 1
        return True
    """

    def kalman_filter(self, pos, Ts):
        """
        卡尔曼滤波核心逻辑
        pos: 观测位置 (cx, cy)，None表示丢失
        Ts: 时间步长（秒）
        return: 更新后的状态 [x, y, vx, vy]
        """
        # 更新状态转移矩阵中的时间步长 Ts
        self.A[0, 2] = Ts
        self.A[1, 3] = Ts

        # 丢包阻尼处理
        damping = 0.94 if pos else 0.82
        self.A[2, 2] = damping
        self.A[3, 3] = damping

        # --- 预测阶段 (Predict) ---
        x_pre = np.dot(self.A, self.x_hat)
        # P_pre = A*P*A.T + Q
        P_pre = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

        # --- 更新阶段 (Update) ---
        if pos:
            self.lost_count = 0
            self.first_detected = True
            z = np.array(pos, dtype=np.float) # [cx, cy]

            # 计算增益 K = P_pre*H.T / (H*P_pre*H.T + R)
            S = np.dot(self.H, np.dot(P_pre, self.H.T)) + self.R
            try:
                K = np.dot(P_pre, np.dot(self.H.T, np.linalg.inv(S)))
            except np.linalg.LinAlgError:
                self.x_hat = x_pre
                self.P = P_pre
                return self.x_hat

            # 更新状态 x_hat = x_pre + K*(z - H*x_pre)
            self.x_hat = x_pre + np.dot(K, (z - np.dot(self.H, x_pre)))
            # 更新协方差 P = (I - K*H)*P_pre
            self.P = np.dot((np.eye(4) - np.dot(K, self.H)), P_pre)
        else:
            self.lost_count += 1
            self.x_hat = x_pre # 丢失时仅依靠预测
            self.P = P_pre

        return self.x_hat

# ======================== 全局状态变量 ========================
# 模式定义
MODE_TARGET = 0          # 目标跟踪模式
MODE_BOUNDARY_UD = 1     # 上下边界矫正模式
MODE_BOUNDARY_LR = 2     # 左右边界矫正模式
MODE_WAITING = 3         # 等待模式
current_mode = MODE_WAITING

# 锁定状态变量
is_target_locked = False        # 是否锁定目标
locked_target_color = ''        # 锁定的目标颜色
locked_target_cx = SCREEN_CENTER_X  # 锁定目标的初始x坐标
locked_target_cy = SCREEN_CENTER_Y  # 锁定目标的初始y坐标
locked_last_cx = SCREEN_CENTER_X    # 上一帧锁定目标的x坐标
locked_last_cy = SCREEN_CENTER_Y    # 上一帧锁定目标的y坐标
locked_lost_count = 0           # 锁定目标丢失帧数

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
        elif cmd == b'U':
            current_mode = MODE_BOUNDARY_UD
        elif cmd == b'L':
            current_mode = MODE_BOUNDARY_LR
        elif cmd == b'F':
            current_mode = MODE_WAITING

# ======================== 初始化 ========================
# 串口初始化
uart = UART(UART_PORT, baudrate=UART_BAUDRATE)
uart.write("uart test\r\n")

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(CAMERA_PIXFORMAT)
sensor.set_framesize(CAMERA_FRAMESIZE)
sensor.set_framerate(CAMERA_FRAMERATE)
sensor.set_auto_gain(False)  # 关闭自动增益
sensor.set_auto_whitebal(False)  # 关闭自动白平衡
sensor.set_brightness(CAMERA_BRIGHTNESS)
# sensor.set_contrast(2) # 对比度
sensor.skip_frames(time=200)  # 跳过初始帧，让摄像头稳定
clock = time.clock()
# LED(4).on

# LCD初始化
lcd = seekfree.IPS200(3)
lcd.full()

# 创建模块实例
color_detector = ColorDetector()
boundary_detector = BoundaryDetector()
brown_tracker = KalmanTracker()
white_tracker = KalmanTracker()
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
        """
        LED(1).on()
        LED(1).off()
        """
        continue

    # 目标跟踪模式
    elif current_mode == MODE_TARGET:
        """
        LED(2).on()
        LED(2).off()
        """
        # LED(4).off()
        # LED(4).on()
        # 色块检测与筛选
        all_blobs_with_color = color_detector.detect_colors(img)
        filtered_blobs_with_color = color_detector.filter_all_blobs(all_blobs_with_color)

        # 初始化变量
        center = []  # 所有有效色块的中心坐标 (cx, cy, color)
        target_pos = None  # 最终要发送的目标坐标
        locked_blob = None  # 锁定的目标色块
        target_color = ''

        # 分离棕色、白色与其它色块
        brown_blobs = []
        white_blobs = []
        other_blobs = []
        for item in filtered_blobs_with_color:
            blob = item[0]
            color = item[1]
            if color == 'brown':
                brown_blobs.append(blob)
            elif color == 'white':
                white_blobs.append(blob)
            else:
                other_blobs.append((blob, color))

        # 处理棕色色块
        if brown_blobs:
            # 选择面积最大的棕色色块
            target_brown = max(brown_blobs, key=lambda b: b.area())
            brown_pos = (target_brown.cx(), target_brown.cy())
            brown_tracker.last_w = target_brown.w()
            brown_tracker.last_h = target_brown.h()

            # 绘制棕色色块
            img.draw_rectangle(target_brown.rect(), color=DRAW_COLORS['brown'])
            img.draw_cross(target_brown.cx(), target_brown.cy(), color=DRAW_COLORS['brown'])

            # 棕色目标卡尔曼滤波
            brown_state = brown_tracker.kalman_filter(brown_pos, Ts)
            if brown_tracker.first_detected and brown_tracker.lost_count < brown_tracker.MAX_LOST_FRAMES:
                kcx, kcy = int(brown_state[0]), int(brown_state[1])
                center.append((kcx, kcy + (target_brown.h() // 2), 'brown'))
                # 绘制卡尔曼预测框
                img.draw_rectangle(
                    kcx - brown_tracker.last_w//2,
                    kcy - brown_tracker.last_h//2,
                    brown_tracker.last_w,
                    brown_tracker.last_h,
                    color=DRAW_COLORS['grey']
                )
                img.draw_cross(kcx, kcy, color=DRAW_COLORS['grey'])
        else:
            # 无棕色色块，仅预测
            brown_tracker.kalman_filter(None, Ts)
            if brown_tracker.lost_count >= brown_tracker.MAX_LOST_FRAMES:
                brown_tracker.reset()

        # 处理白色色块
        if white_blobs:
            # 选择面积最大的白色色块
            target_white = max(white_blobs, key=lambda b: b.area())
            white_pos = (target_white.cx(), target_white.cy())
            white_tracker.last_w = target_white.w()
            white_tracker.last_h = target_white.h()

            # 绘制白色色块
            img.draw_rectangle(target_white.rect(), color=DRAW_COLORS['white'])
            img.draw_cross(target_white.cx(), target_white.cy(), color=DRAW_COLORS['white'])

            # 白色目标卡尔曼滤波
            white_state = white_tracker.kalman_filter(white_pos, Ts)
            if white_tracker.first_detected and white_tracker.lost_count < white_tracker.MAX_LOST_FRAMES:
                kcx, kcy = int(white_state[0]), int(white_state[1])
                center.append((kcx, kcy + (target_white.h() // 2), 'white'))
                # 绘制卡尔曼预测框
                img.draw_rectangle(
                    kcx - white_tracker.last_w//2,
                    kcy - white_tracker.last_h//2,
                    white_tracker.last_w,
                    white_tracker.last_h,
                    color=DRAW_COLORS['grey']
                )
                img.draw_cross(kcx, kcy, color=DRAW_COLORS['grey'])
        else:
            # 无白色色块，仅预测
            white_tracker.kalman_filter(None, Ts)
            if white_tracker.lost_count >= white_tracker.MAX_LOST_FRAMES:
                white_tracker.reset()

        # 处理其他颜色色块
        for item in other_blobs:
            blob = item[0]
            color_name = item[1]
            # 绘制色块
            img.draw_rectangle(blob.rect(), color=DRAW_COLORS[color_name])
            img.draw_cross(blob.cx(), blob.cy(), color=DRAW_COLORS[color_name])
            # 添加到中心列表
            center.append((blob.cx(), blob.cy() + (blob.h() // 2), color_name))

        # 锁定处理逻辑
        if filtered_blobs_with_color:
            # 未锁定：选择y最大的目标作为锁定对象
            if not is_target_locked:
                max_y_blob, max_y_color = max(
                    filtered_blobs_with_color,
                    key=lambda item: item[0].cy()
                )
                locked_target_cx = max_y_blob.cx()
                locked_target_cy = max_y_blob.cy()
                locked_last_cx = locked_target_cx
                locked_last_cy = locked_target_cy
                locked_target_color = max_y_color
                is_target_locked = True
                locked_lost_count = 0
                target_pos = (locked_target_cx, locked_target_cy + (max_y_blob.h() // 2))
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
                    target_pos = (best_cx, best_cy + (best_blob.h() // 2))
                    locked_last_cx = best_cx
                    locked_last_cy = best_cy
                    locked_lost_count = 0
                    target_color = locked_target_color
                    locked_blob = best_blob
                else:
                    # 无有效同色目标，计数+1
                    locked_lost_count += 1
                    target_pos = None
        else:
            # 无任何色块，锁定计数+1
            if is_target_locked:
                locked_lost_count += 1
            target_pos = None

        # 超过最大丢失帧数，解除锁定
        if is_target_locked and locked_lost_count >= LOCK_MAX_LOST_FRAMES:
            reset_lock_state()
            target_color = ''

        # 绘制锁定标识（黑色圆）
        if is_target_locked and locked_blob is not None:
            lock_cx = locked_blob.cx()
            lock_cy = locked_blob.cy()
            img.draw_circle(lock_cx, lock_cy, 5, color=DRAW_COLORS['black'], thickness=2)

        # 发送目标坐标
        if is_target_locked and target_pos is not None:
            # 锁定状态：发送锁定目标坐标
            communicator.send_coordinate(target_pos[0], target_pos[1], target_color)
        elif center:
            # 未锁定：按原有逻辑选y最大的坐标
            target = max(center, key=lambda coordinate: coordinate[1])
            target_x = target[0]
            target_y = target[1]
            target_color = target[2]
            communicator.send_coordinate(target_x, target_y, target_color)

    # 上下边界矫正模式
    elif current_mode == MODE_BOUNDARY_UD:
        """
        LED(3).on()
        LED(3).off()
        """
        angle = boundary_detector.boundary_correction('row', img)
        if angle is not None:
            communicator.send_angle(angle)

    # 左右边界矫正模式
    elif current_mode == MODE_BOUNDARY_LR:
        """
        LED(4).on()
        LED(4).off()
        """
        angle = boundary_detector.boundary_correction('column', img)
        if angle is not None:
            communicator.send_angle(angle)

    # 显示图像到LCD
    lcd.show_image(img, SCREEN_WIDTH, SCREEN_HEIGHT, zoom=0)
    # print(clock.fps())
