import sensor
import image
import time
import tf
from machine import UART
from ulab import numpy as np
import seekfree
import ustruct
from pyb import LED

# ======================== 常量定义 ========================
white = LED(4)  # LED4 照明灯

# 串口配置
UART_PORT = 12
UART_BAUDRATE = 115200

# 摄像头配置
CAMERA_PIXFORMAT = sensor.RGB565
CAMERA_FRAMESIZE = sensor.QQVGA  # 160x120
CAMERA_FRAMERATE = 60
CAMERA_BRIGHTNESS = 600

# 屏幕尺寸
SCREEN_WIDTH = 160
SCREEN_HEIGHT = 120
SCREEN_CENTER_X = SCREEN_WIDTH // 2  # 80
SCREEN_CENTER_Y = SCREEN_HEIGHT // 2  # 60

# 卡尔曼滤波配置
KALMAN_MAX_LOST_FRAMES = 3
MAX_SPEED = 80
JUMP_KALMAN_THRESHOLD = 30  # 卡尔曼预测跳变超过30像素视为异常

# 去噪配置
MIN_DETECT_AREA = 10   # 最小检测面积（像素），低于此视为噪点

# 锁定逻辑配置
LOCK_JUMP_THRESHOLD = 20  # 坐标跳变超过20像素视为同色干扰
LOCK_MAX_LOST_FRAMES = 3  # 丢失3帧解除锁定

# 通信协议常量
PROTOCOL_HEADER1 = 0xA5
PROTOCOL_HEADER2_COORD = 0xA6
PROTOCOL_FOOTER = 0x5B
PROTOCOL_PREVIEW_HEADER = 0x77
PROTOCOL_PREVIEW_FOOTER = 0x78

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

# 颜色阈值
THRESHOLD = {
    'brown':[(11, 60, -9, 14, 6, 69)],
    'red':[(18, 55, 20, 76, -12, 59)],
    'green':[(54, 98, -60, -20, 39, 109)], 
    'blue':[(36, 80, -30, -6, -49, -29)],
    'white':[(59, 100, -32, -10, 5, 43)]
}

# YOLO模型路径
face_detect = '/sd/yolo3_iou_smartcar_final_with_post_processing.tflite'
# 载入模型
net = tf.load(face_detect)

# ======================== 通信模块 ========================
class Communicator:
    def __init__(self, uart):
        """初始化通信器"""
        self.uart = uart

    def send_coordinate(self, x, y, obj_type=''):
        """发送目标检测坐标至下位机

        Args:
            x: 目标中心x坐标（像素）
            y: 目标中心y坐标（像素）
            obj_type: 物体类型名称，用于映射COLOR_TYPE_MAP中的ASCII码
        """
        # 坐标取整
        x = int(round(x))
        y = int(round(y))

        # 获取颜色类型对应的ASCII码
        type_char = COLOR_TYPE_MAP.get(obj_type, 0x00)

        # 打包并发送
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

    def pack_center_data(self, center_list):
        """将多目标检测结果打包为变长协议包并发送

        协议格式: 帧头(0x77) + 物体数量(1B) + (类型+x+y)*N + 尾帧(0x78)
        坐标自动限幅至屏幕分辨率范围内（SCREEN_WIDTH x SCREEN_HEIGHT）

        Args:
            center_list: 检测目标列表，每项为 (cx, cy, color_name) 三元组
        """
        count = len(center_list)
        if count == 0:
            return None
        buf = bytearray(1 + 1 + count * 3 + 1)
        idx = 0

        buf[idx] = PROTOCOL_PREVIEW_HEADER
        idx += 1
        buf[idx] = count
        idx += 1

        for cx, cy, color in center_list:
            buf[idx] = max(0, min(SCREEN_WIDTH, int(cx)))
            idx += 1
            buf[idx] = max(0, min(SCREEN_HEIGHT, int(cy)))
            idx += 1
            buf[idx] = COLOR_TYPE_MAP.get(color, 0x00)
            idx += 1

        buf[idx] = PROTOCOL_PREVIEW_FOOTER
        self.uart.write(buf)

# ======================== 卡尔曼跟踪模块 ========================
class KalmanTracker:
    """6维卡尔曼滤波器，用于目标位置平滑与丢失预测

    状态向量: [cx, cy, w, h, vx, vy]
    检测到目标时：观测更新校正状态
    目标丢失时：纯预测传播，超过MAX_LOST_FRAMES帧后自动重置
    """
    MAX_LOST_FRAMES = KALMAN_MAX_LOST_FRAMES

    def __init__(self):
        # 观测矩阵C（6维单位阵，状态可直接观测）
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
    """YOLO模型检测器，封装模型推理、卡尔曼跟踪和结果绘制

    负责调用TFLite模型进行推理，管理brown/white/blue物体的卡尔曼跟踪，
    并绘制所有检测结果到图像上。
    """

    def __init__(self, net):
        self.net = net

    def detect(self, img):
        """执行YOLO模型检测

        将输入图像缩放到0.75倍后推理，返回归一化坐标的检测结果。
        缩放的目的：在保持检测精度的同时提升推理速度。

        Returns:
            list: 每项为 (x1, y1, x2, y2, label, score) ——坐标均为[0,1]归一化值
        """
        img1 = img.copy(0.75, 1)
        return tf.detect(self.net, img1)

    def process_kalman_color(self, img, objects, tracker, color, Ts, center_list, kalman_coords_dict):
        """模型检测结果的卡尔曼处理（棕/白/蓝）"""
        detected = False
        target_object = None

        if objects:
            target_object = max(objects, key=lambda obj: (obj[2] - obj[0]) * img.width() * (obj[3] - obj[1]) * img.height())
            x1,y1,x2,y2,label,scores = target_object
            x1 = int(x1 * img.width())
            y1 = int(y1 * img.height())
            x2 = int(x2 * img.width())
            y2 = int(y2 * img.height())
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # 绘制原始检测框（不论卡尔曼开关）
            img.draw_rectangle((x1, y1, x2 - x1, y2 - y1), color=DRAW_COLORS[color])
            img.draw_cross(cx, cy, color=DRAW_COLORS[color])

            if kalman_enabled[color]:
                jump_too_large = False
                if tracker.first_detected:
                    distance_squared = (cx - tracker.last_cx)**2 + (cy - tracker.last_cy)**2
                    if distance_squared > (JUMP_KALMAN_THRESHOLD**2):
                        jump_too_large = True

                if not jump_too_large:
                    w, h = x2 - x1, y2 - y1
                    dx_raw = (cx - tracker.last_cx) / Ts
                    dx = max(-MAX_SPEED, min(MAX_SPEED, dx_raw))
                    dy_raw = (cy - tracker.last_cy) / Ts
                    dy = max(-MAX_SPEED, min(MAX_SPEED, dy_raw))
                    Z = np.array([cx, cy, w, h, dx, dy], dtype=np.float)
                    if not tracker.first_detected:
                        tracker.x_hat = np.array([cx, cy, w, h, 0, 0], dtype=np.float)
                        tracker.p = np.diag([10.0, 10.0, 5.0, 5.0, 100.0, 100.0])
                        tracker.first_detected = True
                    tracker.kalman_filter(Z, Ts, True)
                    tracker.last_cx = cx
                    tracker.last_cy = cy

            detected = True

        # 卡尔曼开启时的丢失预测
        if kalman_enabled[color] and not detected and tracker.first_detected:
            if tracker.lost_count < tracker.MAX_LOST_FRAMES:
                tracker.kalman_filter(None, Ts, is_detected=False)
            else:
                tracker.reset()

        # 输出坐标
        if kalman_enabled[color] and tracker.first_detected:
            kcx, kcy = int(tracker.x_hat[0]), int(tracker.x_hat[1])
            kw, kh = max(1, int(tracker.x_hat[2])), max(1, int(tracker.x_hat[3]))
            img.draw_rectangle(kcx - kw//2, kcy - kh//2, kw, kh, color=DRAW_COLORS['grey'])
            img.draw_cross(kcx, kcy, color=DRAW_COLORS['grey'])
            center_list.append((kcx, kcy, color))
            kalman_coords_dict[color] = (kcx, kcy)
        elif detected:
            center_list.append((cx, cy, color))
            kalman_coords_dict[color] = (cx, cy)
        else:
            kalman_coords_dict[color] = (SCREEN_CENTER_X, SCREEN_CENTER_Y)

        return target_object

    def draw_other_objects(self, img, objects, center_list):
        """绘制非卡尔曼跟踪的模型检测结果"""
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

# ======================== 颜色检测模块 ========================
class ColorDetector:
    # 距离阈值（过滤过近的色块）
    DISTANCE_THRESHOLD = 100

    @staticmethod
    def calculate_distance(x1, y1, x2, y2):
        """计算两点间欧氏距离"""
        return (x1 - x2)**2 + (y1 - y2)**2

    def detect_colors(self, img, target_color = ''):
        """检测所有颜色色块并返回（带颜色标签）"""

        # 检测各颜色色块
        if target_color:
            blobs = []
            if target_color == 'brown':
                target_blobs = img.find_blobs(THRESHOLD['brown'], pixels_threshold=100, area_threshold=100, merge=True)
            elif target_color == 'white':
                target_blobs = img.find_blobs(THRESHOLD['white'], pixels_threshold=100, area_threshold=100, merge=True)
            elif target_color == 'red':
                target_blobs   = img.find_blobs(THRESHOLD['red'],   pixels_threshold=110,  area_threshold=110,  merge=True)
            elif target_color == 'green':
                target_blobs = img.find_blobs(THRESHOLD['green'], pixels_threshold=40,  area_threshold=40,  merge=True)
            elif target_color == 'blue':
                target_blobs  = img.find_blobs(THRESHOLD['blue'],  pixels_threshold=40,  area_threshold=40,  merge=True)
            else:
                target_blobs = []

            for blob in target_blobs:blobs.append((blob, target_color))
            return blobs
        else:
            brown_blobs = img.find_blobs(THRESHOLD['brown'], pixels_threshold=100, area_threshold=100, merge=True)
            white_blobs = img.find_blobs(THRESHOLD['white'], pixels_threshold=100, area_threshold=100, merge=True)
            red_blobs   = img.find_blobs(THRESHOLD['red'],   pixels_threshold=110,  area_threshold=110,  merge=True)
            green_blobs = img.find_blobs(THRESHOLD['green'], pixels_threshold=40,  area_threshold=40,  merge=True)
            blue_blobs  = img.find_blobs(THRESHOLD['blue'],  pixels_threshold=40,  area_threshold=40,  merge=True)

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
            if blob.density() < 0.4 and color not in('white', 'brown', 'blue'):
                continue
            elif color in ('white', 'brown', 'blue') and blob.density() < 0.3:
                continue
            elif color == 'green' and blob.density() < 0.45:
                continue

            # 长宽比过滤（不同颜色有不同规则）
            if color == 'brown' and (blob.w() > 3.5 * blob.h() or blob.h() > 3.5 * blob.w()):
                continue
            elif color == 'white' and (blob.w() > 3.5 * blob.h() or blob.h() > 3.5 * blob.w()):
                continue
            elif color == 'green' and (blob.w() > 2 * blob.h() or blob.h() > 2 * blob.w()):
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
        """色块检测结果的卡尔曼处理（棕/白/蓝）"""
        detected = False
        target_blob = None

        if blobs:
            target_blob = max(blobs, key=lambda b: b.area())
            cx, cy = target_blob.cx(), target_blob.cy()

            # 绘制原始检测框（不论卡尔曼开关）
            img.draw_rectangle(target_blob.rect(), color=DRAW_COLORS[color])
            img.draw_cross(cx, cy, color=DRAW_COLORS[color])

            if kalman_enabled[color]:
                # 卡尔曼滤波跟踪
                w, h = target_blob.w(), target_blob.h()
                dx_raw = (cx - tracker.last_cx) / Ts
                dx = max(-MAX_SPEED, min(MAX_SPEED, dx_raw))
                dy_raw = (cy - tracker.last_cy) / Ts
                dy = max(-MAX_SPEED, min(MAX_SPEED, dy_raw))
                Z = np.array([cx, cy, w, h, dx, dy], dtype=np.float)
                if not tracker.first_detected:
                    tracker.x_hat = np.array([cx, cy, w, h, 0, 0], dtype=np.float)
                    tracker.p = np.diag([10.0, 10.0, 5.0, 5.0, 100.0, 100.0])
                    tracker.first_detected = True
                tracker.kalman_filter(Z, Ts, True)
                tracker.last_cx = cx
                tracker.last_cy = cy

            detected = True

        # 卡尔曼开启时的丢失预测
        if kalman_enabled[color] and not detected and tracker.first_detected:
            if tracker.lost_count < tracker.MAX_LOST_FRAMES:
                tracker.kalman_filter(None, Ts, is_detected=False)
            else:
                tracker.reset()

        # 输出坐标
        if kalman_enabled[color] and tracker.first_detected:
            kcx, kcy = int(tracker.x_hat[0]), int(tracker.x_hat[1])
            kw, kh = max(1, int(tracker.x_hat[2])), max(1, int(tracker.x_hat[3]))
            img.draw_rectangle(kcx - kw//2, kcy - kh//2, kw, kh, color=DRAW_COLORS['grey'])
            img.draw_cross(kcx, kcy, color=DRAW_COLORS['grey'])
            center_list.append((kcx, kcy, color))
            kalman_coords_dict[color] = (kcx, kcy)
        elif detected:
            center_list.append((cx, cy, color))
            kalman_coords_dict[color] = (cx, cy)
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

# ======================== 全局状态变量 ========================

# 运行模式定义（通过UART命令切换）
MODE_COLOR = 0           # 颜色模式：检测色块→锁定目标→发送单个目标
MODE_MODEL = 1           # 模型模式：检测物体→卡尔曼跟踪→发送单个目标
MODE_PREVIEW = 2         # 预览模式：检测所有物体→打包发送（上位机完整展示）
MODE_WAITING = 3         # 等待模式：空闲，仅维持摄像头画面显示
current_mode = MODE_WAITING

# 各颜色对应的卡尔曼预测坐标（供TargetLocker使用），默认值为屏幕中心
kalman_coords = {
    'brown': (SCREEN_CENTER_X, SCREEN_CENTER_Y),
    'white': (SCREEN_CENTER_X, SCREEN_CENTER_Y),
    'blue': (SCREEN_CENTER_X, SCREEN_CENTER_Y)
}

# 卡尔曼滤波开关（关闭后使用原始检测坐标）
kalman_enabled = {
    'brown': True,
    'white': True,
    'blue': True
}

# 上一帧的时间戳，用于计算卡尔曼滤波的时间步长Ts
last_time = time.ticks_ms()

# ======================== 工具函数 ========================
# 当前选中目标类型（由下位机通过UART指定）
current_obj = ''

def handle_uart_commands(uart):
    """处理来自下位机的串口命令：切换运行模式 OR 更新当前目标物体类型

    模式切换命令:
        C → MODE_COLOR (色块模式)
        M → MODE_MODEL (目标检测与跟踪)
        A → MODE_PREVIEW (全量数据预览)
        F → MODE_WAITING (空闲等待)

    物体类型命令:
        b → 棕色熊(brown)
        w → 白色熊(white)
        s → 红色沙包(red)
        e → 蓝色沙包(blue)
        t → 绿色网球(green)
    """
    global current_mode, current_obj
    if uart.any():
        cmd = uart.read(1)

        def reset_all():
            """重置各跟踪器状态，确保模式切换时轨迹不混叠"""
            brown_tracker.reset()
            white_tracker.reset()
            blue_tracker.reset()

        # ---------- 模式切换命令 ----------
        if cmd == b'C':
            current_mode = MODE_COLOR
            reset_all()
        elif cmd == b'M':
            current_mode = MODE_MODEL
            reset_all()
        elif cmd == b'A':
            current_mode = MODE_PREVIEW
            reset_all()
        elif cmd == b'F':
            current_mode = MODE_WAITING
            reset_all()

        # ---------- 物体类型命令（全局可用） ----------
        elif cmd in (b's', b'e', b't', b'b', b'w'):
            color_map = {
                b's': 'red',
                b'e': 'blue',
                b't': 'green',
                b'b': 'brown',
                b'w': 'white'
            }
            current_obj = color_map[cmd]
            if current_mode == MODE_COLOR:
                reset_all()
        elif cmd == b'c':
            current_obj = ''
            if current_mode == MODE_COLOR:
                reset_all()


def detect_all_objects(img, Ts):
    """执行全量目标检测，按颜色分类处理并汇总结果

    处理流程:
        1. YOLO模型推理，获取所有检测框
        2. brown/white/blue物体 → 卡尔曼滤波跟踪（置信度阈值0.3）
        3. red/green物体 → 直接绘制检测框（置信度阈值0.5，不做卡尔曼跟踪）
        4. 所有检测到的目标坐标（含卡尔曼预测值）汇总到center列表

    Args:
        img: 当前帧图像（会被process_kalman_color和draw_other_objects修改）
        Ts: 帧间隔时间（秒）

    Returns:
        center: 所有检测目标列表 [(cx, cy, color_name), ...]
        objects: YOLO原始检测结果
    """
    center = []
    objects = model_detector.detect(img)
    objects = [
    obj for obj in objects
    if (obj[2] - obj[0]) * img.width() * (obj[3] - obj[1]) * img.height() >= MIN_DETECT_AREA
]

    # 按颜色分类过滤（不同颜色置信度阈值不同）
    brown_bear = [obj for obj in objects if LABEL_TO_COLOR.get(obj[4]) == 'brown' and obj[5] > 0.3]
    white_bear = [obj for obj in objects if LABEL_TO_COLOR.get(obj[4]) == 'white' and obj[5] > 0.3]
    blue_bear = [obj for obj in objects if LABEL_TO_COLOR.get(obj[4]) == 'blue' and obj[5] > 0.3]

    other_objects = []
    for obj in objects:
        color = LABEL_TO_COLOR.get(obj[4])
        confidence = obj[5]
        if color in ['red', 'green'] and confidence > 0.5:
            other_objects.append((obj, color))

    # brown/white/blue使用卡尔曼跟踪（含丢失预测），其他颜色直接绘制
    model_detector.process_kalman_color(img, brown_bear, brown_tracker, 'brown', Ts, center, kalman_coords)
    model_detector.process_kalman_color(img, white_bear, white_tracker, 'white', Ts, center, kalman_coords)
    model_detector.process_kalman_color(img, blue_bear, blue_tracker, 'blue', Ts, center, kalman_coords)
    model_detector.draw_other_objects(img, other_objects, center)

    return center, objects

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
sensor.set_vflip(True)
sensor.skip_frames(time=200)  # 跳过初始帧，让摄像头稳定
sensor.set_hmirror(True)
sensor.skip_frames(time=200)  # 跳过初始帧，让摄像头稳定
clock = time.clock()

# LCD初始化
lcd = seekfree.IPS200(3)
lcd.full()

# 创建各模块实例
communicator = Communicator(uart)
brown_tracker = KalmanTracker()
white_tracker = KalmanTracker()
blue_tracker = KalmanTracker()
model_detector = ModelDetector(net)
color_detector = ColorDetector()
target_locker = TargetLocker(LOCK_JUMP_THRESHOLD, LOCK_MAX_LOST_FRAMES)

# ======================== 主循环 ========================
while True:
    clock.tick()
    img = sensor.snapshot()

    # 计算帧间隔时间Ts（秒），用于卡尔曼滤波速度计算
    # 限制最小值0.01s防止Ts=0导致除零或速度发散
    current_time = time.ticks_ms()
    delta_time = time.ticks_diff(current_time, last_time)
    Ts = max(delta_time / 1000.0, 0.01)
    last_time = current_time

    # 处理串口命令（可切换运行模式或设置目标类型）
    handle_uart_commands(uart)

    # 等待模式：不做任何检测或发送，仅维持LCD显示
    if current_mode == MODE_WAITING:
        continue

    # 色块模式：检测色块→锁定目标→发送单个目标
    elif current_mode == MODE_COLOR:
        all_blobs_with_color = color_detector.detect_colors(img, current_obj)
        filtered_blobs_with_color = color_detector.filter_all_blobs(all_blobs_with_color)

        center = []
        target_pos = None
        locked_blob = None

        # 分离棕/白/蓝色块（卡尔曼跟踪）与其他色块（直接绘制）
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

        color_detector.process_kalman_color(img, brown_blobs, brown_tracker, 'brown', Ts, center, kalman_coords)
        color_detector.process_kalman_color(img, white_blobs, white_tracker, 'white', Ts, center, kalman_coords)
        color_detector.process_kalman_color(img, blue_blobs, blue_tracker, 'blue', Ts, center, kalman_coords)
        color_detector.draw_other_blobs(img, other_blobs, center)

        target_pos, target_color, locked_blob = target_locker.process_lock(filtered_blobs_with_color, kalman_coords)
        target_locker.draw_lock_mark(img, locked_blob, kalman_coords)

        # 发送目标坐标
        if target_locker.is_locked and target_pos is not None:
            communicator.send_coordinate(target_pos[0], target_pos[1], target_locker.locked_color)
        elif not target_locker.is_locked and center:
            target = max(center, key=lambda coordinate: coordinate[1])
            communicator.send_coordinate(target[0], target[1], target[2])

    # 模型模式：YOLO检测→卡尔曼跟踪→发送单个目标
    elif current_mode == MODE_MODEL:
        center, _ = detect_all_objects(img, Ts)
        is_sent = False

        # 优先匹配已选类型，否则取最下方物体
        if center:
            matched = [c for c in center if c[2] == current_obj]
            if matched:
                target = max(matched, key=lambda c: c[1])
            else:
                target = max(center, key=lambda c: c[1])
            communicator.send_coordinate(target[0], target[1], target[2])
            is_sent = True

        displayed_text = 'YES' if is_sent else 'NO'
        displayed_text_color = DRAW_COLORS['green'] if is_sent else DRAW_COLORS['red']
        img.draw_string(5, 5, displayed_text, color=displayed_text_color, scale=2)

    # 预览模式：色块检测全量打包发送（供上位机展示）
    elif current_mode == MODE_PREVIEW:
        all_blobs_with_color = color_detector.detect_colors(img, current_obj)
        filtered_blobs_with_color = color_detector.filter_all_blobs(all_blobs_with_color)

        center = []

        # 分离棕/白/蓝色块（卡尔曼跟踪）与其他色块
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

        color_detector.process_kalman_color(img, brown_blobs, brown_tracker, 'brown', Ts, center, kalman_coords)
        color_detector.process_kalman_color(img, white_blobs, white_tracker, 'white', Ts, center, kalman_coords)
        color_detector.process_kalman_color(img, blue_blobs, blue_tracker, 'blue', Ts, center, kalman_coords)
        color_detector.draw_other_blobs(img, other_blobs, center)

        communicator.pack_center_data(center)

    # 显示图像到LCD
    lcd.show_image(img, SCREEN_WIDTH, SCREEN_HEIGHT, zoom=0)