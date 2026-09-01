import sensor
import image
import time
import math
import tf
from machine import UART
from ulab import numpy as np
import seekfree
import ustruct
from pyb import LED
import gc

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
# 白天用800，晚上用600

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

# 发送目标基准点（取坐标离此点最近的目标发送）
DATUM_POINT = (80, 85)

# 多目标跟踪配置
MULTI_MATCH_DISTANCE = 30  # 同一颜色两个物体的匹配距离阈值（像素）
MAX_TRACKERS_PER_COLOR = 2  # 每色最大跟踪器数量，超过按面积舍去最小的

# 通信协议常量
PROTOCOL_HEADER1 = 0xA5
PROTOCOL_HEADER2_COORD = 0xA6
PROTOCOL_FOOTER = 0x5B
PROTOCOL_HEADER2_APRILTAG = 0xA8

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
    'brown': (150, 75, 0)    # 棕色玩具熊
}

# 颜色阈值
# 白天 亮度800
# THRESHOLD = {
#     'brown':[(21, 46, -12, 20, -6, 52), (21, 40, -16, 8, -2, 44)],
#     'red':[(29, 52, 12, 57, -25, 28), (21, 48, 10, 53, -9, 32)],
#     'green':[(51, 97, -48, -13, 38, 95), (41, 98, -52, -21, 5, 93)],
#     'blue':[(38, 77, -24, -5, -55, -27), (29, 40, -21, 2, -41, -24), (27, 39, -15, 1, -34, -18)],
#     'white':[(55, 100, -29, -5, -1, 37), (45, 100, -16, -1, -10, 20)]
# }
# 晚上 亮度600
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

    def send_coordinate_with_angle(self, tag_cx, tag_cy, rotation):
        """发送Apriltag坐标及偏转角"""
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
    """6维卡尔曼滤波器，用于目标位置平滑与丢失预测

    状态向量: [cx, cy, w, h, vx, vy]
    检测到目标时：观测更新校正状态
    目标丢失时：纯预测传播，超过MAX_LOST_FRAMES帧后自动重置
    """
    MAX_LOST_FRAMES = KALMAN_MAX_LOST_FRAMES

    def __init__(self):
        try:
            self.C = np.array([
                [1,0,0,0,0,0],
                [0,1,0,0,0,0],
                [0,0,1,0,0,0],
                [0,0,0,1,0,0],
                [0,0,0,0,1,0],
                [0,0,0,0,0,1]
            ], dtype=np.float)
            self.R = np.diag([0.5, 0.5, 1.5, 1.5, 2, 2])
            self.I = np.eye(6)
            self.Q_detected = np.diag([0.02, 0.02, 0.02, 0.02, 0.2, 0.2])
            self.Q_lost = np.diag([1.0, 1.0, 0.5, 0.5, 2.0, 2.0])
            self.A = np.eye(6)
            self.Z_buf = np.zeros(6)
            self.reset()
        except Exception:
            self.x_hat = None
            self.p = None

    def reset(self):
        """重置跟踪器状态"""
        self.first_detected = False
        self.lost_count = 0
        self.last_cx, self.last_cy = SCREEN_CENTER_X, SCREEN_CENTER_Y
        self.x_hat = np.array([SCREEN_CENTER_X, SCREEN_CENTER_Y, 30, 30, 2, 2], dtype=np.float)
        self.p = np.diag([100.0, 100.0, 50.0, 50.0, 300.0, 300.0])

    def _update_A(self, Ts, damping):
        self.A[0, 4] = Ts
        self.A[1, 5] = Ts
        self.A[4, 4] = damping
        self.A[5, 5] = damping

    def kalman_filter(self, Z, Ts, is_detected):
        """
        卡尔曼滤波核心逻辑
        Z: 观测值数组 [cx, cy, w, h, dx, dy]（None表示丢失）
        Ts: 时间步长（秒）
        is_detected: 是否检测到目标
        return: 更新后的状态 [x, y, w, h, vx, vy]
        """
        if not hasattr(self, 'x_hat') or self.x_hat is None:
            self.reset()
        if not hasattr(self, 'p') or self.p is None:
            self.reset()

        try:
            if is_detected:
                damping = 0.98
                self.lost_count = 0
            else:
                damping = 0.9
                self.lost_count += 1

            Q = self.Q_detected if is_detected else self.Q_lost
            self._update_A(Ts, damping)

            x_hat_minus = np.dot(self.A, self.x_hat)
            p_minus = np.dot(self.A, np.dot(self.p, self.A.T)) + Q

            if is_detected:
                S = np.dot(np.dot(self.C, p_minus), self.C.T) + self.R
                S_reg = S + 1e-4 * self.I
                try:
                    S_inv = np.linalg.inv(S_reg)
                except Exception:
                    diag_vals = []
                    for i in range(S_reg.shape[0]):
                        val = S_reg[i, i]
                        diag_vals.append(1.0 / val if abs(val) > 1e-6 else 1e6)
                    S_inv = np.diag(diag_vals)
                K = np.dot(np.dot(p_minus, self.C.T), S_inv)
                self.x_hat = x_hat_minus + np.dot(K, (Z - np.dot(self.C, x_hat_minus)))
                self.p = np.dot((self.I - np.dot(K, self.C)), p_minus)
            else:
                self.x_hat = x_hat_minus
                self.p = p_minus

            self.x_hat[0] = max(0, min(SCREEN_WIDTH, self.x_hat[0]))
            self.x_hat[1] = max(0, min(SCREEN_HEIGHT, self.x_hat[1]))
            self.x_hat[2] = max(1, min(SCREEN_WIDTH, self.x_hat[2]))
            self.x_hat[3] = max(1, min(SCREEN_HEIGHT, self.x_hat[3]))
            self.x_hat[4] = max(-MAX_SPEED, min(MAX_SPEED, self.x_hat[4]))
            self.x_hat[5] = max(-MAX_SPEED, min(MAX_SPEED, self.x_hat[5]))

            return self.x_hat
        except Exception:
            self.reset()
            return self.x_hat

# ======================== 多目标跟踪模块 ========================
class TrackedObject:
    """单个被跟踪物体的实例，包裹一个卡尔曼滤波器"""
    # 类默认值（防止MicroPython长时间运行后内存碎片导致属性丢失）
    kalman = None
    last_cx = SCREEN_CENTER_X
    last_cy = SCREEN_CENTER_Y
    last_area = 0
    lost_count = 0
    age = 0
    id = 0

    def __init__(self, kalman, cx, cy, area):
        self.kalman = kalman       # 专属 KalmanTracker
        self.last_cx = cx          # 上次匹配的原始检测 x（用于帧间匹配）
        self.last_cy = cy          # 上次匹配的原始检测 y
        self.last_area = area      # 上次匹配的物体面积（用于限制时舍去）
        self.lost_count = 0        # 连续丢失帧数
        self.age = 0               # 存活帧数
        self.id = 0                # 唯一编号


class MultiTracker:
    """管理单个颜色的多个卡尔曼跟踪器——支持同色多物体跟踪

    每帧调用 update() 传入当前检测到的所有该颜色色块坐标，
    内部通过最近邻匹配将检测分配给已有跟踪器，未匹配的检测新建跟踪器。
    """

    def __init__(self, match_distance=MULTI_MATCH_DISTANCE, max_lost=KALMAN_MAX_LOST_FRAMES, max_trackers=MAX_TRACKERS_PER_COLOR):
        self.objects = []               # list[TrackedObject]
        self.match_distance = match_distance  # 匹配距离阈值（像素）
        self.max_lost = max_lost         # 最大丢失帧数
        self.max_trackers = max_trackers # 每色最大跟踪器数量
        self.next_id = 0                 # 下一个跟踪器编号

    def reset(self):
        """重置所有跟踪器"""
        self.objects = []
        self.next_id = 0

    def _repair_kalman(self, obj, cx=SCREEN_CENTER_X, cy=SCREEN_CENTER_Y, w=30, h=30):
        """检查并修复损坏的卡尔曼跟踪器（防止内存问题导致对象丢失）"""
        kalman = getattr(obj, 'kalman', None)
        if not isinstance(kalman, KalmanTracker) or not hasattr(kalman, 'x_hat') or kalman.x_hat is None:
            obj.kalman = KalmanTracker()
            if not hasattr(obj.kalman, 'x_hat') or obj.kalman.x_hat is None:
                return
            obj.kalman.x_hat = np.array([cx, cy, w, h, 0, 0], dtype=np.float)
            obj.kalman.p = np.diag([10.0, 10.0, 5.0, 5.0, 100.0, 100.0])
            obj.kalman.first_detected = True
            obj.lost_count = 0

    def update(self, detections, Ts, kalman_on):
        """执行一帧的多目标跟踪

        Args:
            detections: [(cx, cy, w, h), ...]  当前帧检测到的所有该颜色目标坐标
            Ts: 帧间隔时间（秒）
            kalman_on: 当前颜色的卡尔曼开关

        Returns:
            [(kcx, kcy, kw, kh, raw_cx, raw_cy), ...]
            每个活跃跟踪器的输出坐标（kcx/kcy 是卡尔曼预测/原始坐标，
            raw_cx/raw_cy 是原始检测坐标）
        """
        results = []

        # 情况1：没有检测也没有跟踪器 → 空
        if not detections and not self.objects:
            return results

        # 情况2：没有检测 → 所有跟踪器做预测（或标记丢失）
        if not detections:
            for obj in self.objects:
                self._repair_kalman(obj)
                if kalman_on and obj.lost_count < self.max_lost:
                    obj.kalman.kalman_filter(None, Ts, is_detected=False)
                    obj.lost_count += 1
                    kcx, kcy, kw, kh = self._get_kalman_output(obj)
                    results.append((kcx, kcy, kw, kh, kcx, kcy))
                else:
                    obj.lost_count += 1
            self._cleanup()
            return results

        # 情况3：有检测 → 最近邻匹配
        used = set()  # 已匹配的检测索引

        # 第1步：每个已有跟踪器找最近的未匹配检测
        for obj in self.objects:
            best_di = None
            best_dist = self.match_distance ** 2

            for di, (dcx, dcy, _, _) in enumerate(detections):
                if di in used:
                    continue
                dist = (dcx - obj.last_cx)**2 + (dcy - obj.last_cy)**2
                if dist < best_dist:
                    best_dist = dist
                    best_di = di

            if best_di is not None:
                # 匹配成功 → 卡尔曼更新
                dcx, dcy, dw, dh = detections[best_di]
                used.add(best_di)
                self._repair_kalman(obj, dcx, dcy, dw, dh)

                if kalman_on:
                    dx = max(-MAX_SPEED, min(MAX_SPEED, (dcx - obj.last_cx) / Ts))
                    dy = max(-MAX_SPEED, min(MAX_SPEED, (dcy - obj.last_cy) / Ts))
                    Z_buf = obj.kalman.Z_buf
                    Z_buf[0] = dcx; Z_buf[1] = dcy; Z_buf[2] = dw; Z_buf[3] = dh
                    Z_buf[4] = dx; Z_buf[5] = dy
                    Z = Z_buf
                    if not obj.kalman.first_detected:
                        obj.kalman.x_hat = np.array([dcx, dcy, dw, dh, 0, 0], dtype=np.float)
                        obj.kalman.p = np.diag([10.0, 10.0, 5.0, 5.0, 100.0, 100.0])
                        obj.kalman.first_detected = True
                    obj.kalman.kalman_filter(Z, Ts, True)
                    kcx, kcy = int(obj.kalman.x_hat[0]), int(obj.kalman.x_hat[1])
                    kw = max(1, int(obj.kalman.x_hat[2]))
                    kh = max(1, int(obj.kalman.x_hat[3]))
                    results.append((kcx, kcy, kw, kh, dcx, dcy))
                else:
                    results.append((dcx, dcy, dw, dh, dcx, dcy))

                obj.last_cx, obj.last_cy = dcx, dcy
                obj.last_area = dw * dh
                obj.lost_count = 0
                obj.age += 1
            else:
                # 未匹配到检测 → 丢失帧
                self._repair_kalman(obj)
                if kalman_on and obj.lost_count < self.max_lost:
                    obj.kalman.kalman_filter(None, Ts, is_detected=False)
                    obj.lost_count += 1
                    kcx, kcy, kw, kh = self._get_kalman_output(obj)
                    results.append((kcx, kcy, kw, kh, kcx, kcy))
                else:
                    obj.lost_count += 1

        # 第2步：未匹配的检测 → 新建跟踪器
        for di, (dcx, dcy, dw, dh) in enumerate(detections):
            if di not in used:
                kalman = KalmanTracker()
                kalman.x_hat = np.array([dcx, dcy, dw, dh, 0, 0], dtype=np.float)
                kalman.p = np.diag([10.0, 10.0, 5.0, 5.0, 100.0, 100.0])
                kalman.first_detected = True

                new_obj = TrackedObject(kalman, dcx, dcy, dw * dh)
                new_obj.age = 1
                new_obj.id = self.next_id
                self.next_id += 1
                self.objects.append(new_obj)

                results.append((dcx, dcy, dw, dh, dcx, dcy))

        # 第3步：清理超时跟踪器
        self._cleanup()

        # 第4步：限制跟踪器数量，超过上限按面积舍去最小的
        while len(self.objects) > self.max_trackers:
            smallest = min(self.objects, key=lambda o: o.last_area)
            self.objects.remove(smallest)

        return results

    def _get_kalman_output(self, obj):
        """从卡尔曼状态中提取预测框（含NaN保护）"""
        if obj is None or getattr(obj, 'kalman', None) is None:
            return SCREEN_CENTER_X, SCREEN_CENTER_Y, 30, 30
        try:
            kcx = int(obj.kalman.x_hat[0])
            kcy = int(obj.kalman.x_hat[1])
            kw = max(1, int(obj.kalman.x_hat[2]))
            kh = max(1, int(obj.kalman.x_hat[3]))
        except (ValueError, OverflowError, AttributeError, TypeError):
            kcx, kcy = obj.last_cx, obj.last_cy
            kw = kh = 30
            try:
                obj.kalman = KalmanTracker()
            except Exception:
                obj.kalman = None
        return kcx, kcy, kw, kh

    def _cleanup(self):
        """移除超时的跟踪器"""
        self.objects = [o for o in self.objects if o.lost_count < self.max_lost]

    @property
    def count(self):
        return len(self.objects)


# ======================== 模型检测模块  ========================
class ModelDetector:
    """YOLO模型检测器，封装模型推理、多目标卡尔曼跟踪和结果绘制

    负责调用TFLite模型进行推理，管理brown/white/blue物体的多目标卡尔曼跟踪，
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

    def process_kalman_multi(self, img, objects, multi_tracker, color, Ts, center_list, kalman_coords_dict):
        """模型检测结果的多目标卡尔曼处理

        对所有检测到的物体执行多目标跟踪，绘制所有原始框，
        然后对每个跟踪器绘制灰色预测框并输出坐标。
        """
        width, height = img.width(), img.height()
        kalman_on = kalman_enabled[color]

        # 1. 提取检测坐标并绘制原始检测框
        detections = []
        for obj in objects:
            x1_n, y1_n, x2_n, y2_n, label, scores = obj
            x1 = int(x1_n * width)
            y1 = int(y1_n * height)
            x2 = int(x2_n * width)
            y2 = int(y2_n * height)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            w = x2 - x1
            h = y2 - y1

            img.draw_rectangle((x1, y1, w, h), color=DRAW_COLORS[color])
            img.draw_cross(cx, cy, color=DRAW_COLORS[color])
            detections.append((cx, cy, w, h))

        # 2. 多目标跟踪
        results = multi_tracker.update(detections, Ts, kalman_on)

        # 3. 绘制预测框并输出坐标
        for kcx, kcy, kw, kh, raw_cx, raw_cy in results:
            if kalman_on and (kcx != raw_cx or kcy != raw_cy):
                img.draw_rectangle(kcx - kw//2, kcy - kh//2, kw, kh, color=DRAW_COLORS['grey'])
                img.draw_cross(kcx, kcy, color=DRAW_COLORS['grey'])
                center_list.append((kcx, kcy, color))
            else:
                center_list.append((raw_cx, raw_cy, color))

        # 4. 更新 kalman_coords_dict
        if results:
            if kalman_on:
                lowest = max([(r[0], r[1]) for r in results], key=lambda p: p[1])
            else:
                lowest = max([(r[4], r[5]) for r in results], key=lambda p: p[1])
            kalman_coords_dict[color] = lowest
        else:
            kalman_coords_dict[color] = (SCREEN_CENTER_X, SCREEN_CENTER_Y)

    def draw_other_objects(self, img, objects, center_list):
        """绘制非卡尔曼跟踪的模型检测结果"""
        for item in objects:
            obj, color_name = item
            x1_n, y1_n, x2_n, y2_n, label, scores = obj
            x1 = int(x1_n * img.width())
            y1 = int(y1_n * img.height())
            x2 = int(x2_n * img.width())
            y2 = int(y2_n * img.height())
            w = x2 - x1
            h = y2 - y1
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            img.draw_rectangle((x1, y1, w, h), color=DRAW_COLORS[color_name])
            img.draw_cross(cx, cy, color=DRAW_COLORS[color_name])
            center_list.append((cx, cy, color_name))

# ======================== 颜色检测模块 ========================
class ColorDetector:
    # 距离阈值（过滤过近的色块）
    DISTANCE_THRESHOLD = 65

    @staticmethod
    def calculate_distance(x1, y1, x2, y2):
        """计算两点间欧氏距离"""
        return (x1 - x2)**2 + (y1 - y2)**2

    def detect_colors(self, img, target_color='', use_preview_threshold=False):
        """检测所有颜色色块并返回（带颜色标签）

        Args:
            img: 输入图像
            target_color: 指定要检测的目标颜色（空字符串表示检测全部）
            use_preview_threshold: True=预览模式(红绿蓝=10，棕白=25)，False=色块模式
        """
        if use_preview_threshold:
            threshold_map = {
                'red':   (10, 10),
                'green': (10, 10),
                'blue':  (20, 20),
                'brown': (20, 20),
                'white': (25, 25),
            }
        else:
            threshold_map = {
                'brown': (100, 100),
                'white': (100, 100),
                'red':   (100, 100),
                'green': (40, 40),
                'blue':  (40, 40),
            }

        if target_color:
            blobs = []
            if target_color in threshold_map:
                pt, at = threshold_map[target_color]
                target_blobs = img.find_blobs(THRESHOLD[target_color], pixels_threshold=pt, area_threshold=at, merge=True)
            else:
                target_blobs = []

            for blob in target_blobs:
                blobs.append((blob, target_color))
            return blobs
        else:
            all_blobs = []
            for color, (pt, at) in threshold_map.items():
                found = img.find_blobs(THRESHOLD[color], pixels_threshold=pt, area_threshold=at, merge=True)
                for blob in found:
                    all_blobs.append((blob, color))
            return all_blobs

    def filter_all_blobs(self, blobs):
        """过滤无效色块（密度、像素数、长宽比、距离）"""
        filtered = []
        for blob, color in blobs:
            if blob.density() < 0.4 and color not in('white', 'brown', 'blue'):
                continue
            elif color in ('white', 'brown', 'blue') and blob.density() < 0.3:
                continue
            elif color == 'green' and blob.density() < 0.45:
                continue

            if color == 'brown' and (blob.w() > 3.5 * blob.h() or blob.h() > 3.5 * blob.w()):
                continue
            elif color == 'white' and (blob.w() > 3.5 * blob.h() or blob.h() > 3.5 * blob.w()):
                continue
            elif color == 'green' and (blob.w() > 2 * blob.h() or blob.h() > 2 * blob.w()):
                continue
            elif color == 'red' and (blob.w() > 2 * blob.h() or blob.h() > 2 * blob.w()):
                continue
            elif color == 'blue' and (blob.w() > 1.5 * blob.h() or blob.h() > 1.5 * blob.w()):
                continue

            cx, cy = blob.cx(), blob.cy()
            keep = True
            for saved_blob, saved_color in filtered:
                if saved_color != color:
                    continue
                if self.calculate_distance(cx, cy, saved_blob.cx(), saved_blob.cy()) < self.DISTANCE_THRESHOLD:
                    keep = False
                    break
            if keep:
                filtered.append((blob, color))
        return filtered

    def process_kalman_multi(self, img, blobs, multi_tracker, color, Ts, center_list, kalman_coords_dict):
        """色块检测结果的多目标卡尔曼处理

        对所有检测到的色块执行多目标跟踪，绘制所有原始框，
        然后对每个跟踪器绘制灰色预测框并输出坐标。
        """
        kalman_on = kalman_enabled[color]

        # 1. 绘制所有原始检测框
        for blob in blobs:
            img.draw_rectangle(blob.rect(), color=DRAW_COLORS[color])
            img.draw_cross(blob.cx(), blob.cy(), color=DRAW_COLORS[color])

        # 2. 提取检测坐标
        detections = [(b.cx(), b.cy(), b.w(), b.h()) for b in blobs] if blobs else []

        # 3. 多目标跟踪
        results = multi_tracker.update(detections, Ts, kalman_on)

        # 4. 绘制预测框并输出坐标
        for kcx, kcy, kw, kh, raw_cx, raw_cy in results:
            if kalman_on and (kcx != raw_cx or kcy != raw_cy):
                img.draw_rectangle(kcx - kw//2, kcy - kh//2, kw, kh, color=DRAW_COLORS['grey'])
                img.draw_cross(kcx, kcy, color=DRAW_COLORS['grey'])
                center_list.append((kcx, kcy, color))
            else:
                center_list.append((raw_cx, raw_cy, color))

        # 5. 更新 kalman_coords_dict
        if results:
            if kalman_on:
                lowest = max([(r[0], r[1]) for r in results], key=lambda p: p[1])
            else:
                lowest = max([(r[4], r[5]) for r in results], key=lambda p: p[1])
            kalman_coords_dict[color] = lowest
        else:
            kalman_coords_dict[color] = (SCREEN_CENTER_X, SCREEN_CENTER_Y)

    def draw_other_blobs(self, img, blobs, center_list):
        """封装绘制其他颜色色块逻辑"""
        for item in blobs:
            blob = item[0]
            color_name = item[1]
            img.draw_rectangle(blob.rect(), color=DRAW_COLORS[color_name])
            img.draw_cross(blob.cx(), blob.cy(), color=DRAW_COLORS[color_name])
            center_list.append((blob.cx(), blob.cy(), color_name))


# ======================== 坐标矫正模块 ========================
class CoordinateCorrection:
    def __init__(self):
        self.tag_family = image.TAG25H9

    def coordinate_correction(self, img):
        """寻找并标记Apriltag，计算实际偏转角度"""
        tags = img.find_apriltags(families=self.tag_family)
        if tags:
            tag = tags[0]
            tag_cx = tag.cx()
            tag_cy = tag.cy()
            rotation = (math.acos(math.cos(tag.y_rotation()) * math.cos(tag.z_rotation())) - math.pi / 2)
            img.draw_rectangle(tag.rect(), color=(255, 0, 0))
            img.draw_cross(tag_cx, tag_cy, color=(0, 255, 0))
            return (tag_cx, tag_cy, rotation)
        return None


# ======================== 全局状态变量 ========================

# 运行模式定义（通过UART命令切换）
MODE_COLOR = 0           # 色块模式：检测色块→多目标卡尔曼→发送单个目标
MODE_MODEL = 1           # 模型模式：YOLO检测→多目标卡尔曼→发送单个目标
MODE_CORRECTION = 3      # 矫正模式：检测Apriltag并发送偏转角
MODE_WAITING = 4         # 等待模式：空闲，仅维持摄像头画面显示
current_mode = MODE_WAITING

# 各颜色对应的卡尔曼预测坐标，默认值为屏幕中心
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

# GC回收配置
GC_INTERVAL = 30

# GC帧计数器
frame_count = 0

# ======================== 工具函数 ========================
# 当前选中目标类型（由下位机通过UART指定）
current_obj = ''

def handle_uart_commands(uart):
    """处理来自下位机的串口命令：切换运行模式 OR 更新当前目标物体类型

    模式切换命令:
        C → MODE_COLOR (色块模式)
        M → MODE_MODEL (目标检测与跟踪)
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
            # sensor.set_brightness(800)
            reset_all()
        elif cmd == b'M':
            current_mode = MODE_MODEL
            # sensor.set_brightness(800)
            reset_all()
        elif cmd == b'R':
            current_mode = MODE_CORRECTION
            # sensor.set_brightness(600)
            reset_all()
        elif cmd == b'F':
            current_mode = MODE_WAITING
            current_obj = ''
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
        2. brown/white/blue物体 → 多目标卡尔曼跟踪（置信度阈值0.3）
        3. red/green物体 → 直接绘制检测框（置信度阈值0.5，不做卡尔曼跟踪）
        4. 所有检测到的目标坐标（含卡尔曼预测值）汇总到center列表

    Args:
        img: 当前帧图像
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

    brown_bear = [obj for obj in objects if LABEL_TO_COLOR.get(obj[4]) == 'brown' and obj[5] > 0.3]
    white_bear = [obj for obj in objects if LABEL_TO_COLOR.get(obj[4]) == 'white' and obj[5] > 0.3]
    blue_bear = [obj for obj in objects if LABEL_TO_COLOR.get(obj[4]) == 'blue' and obj[5] > 0.3]

    other_objects = []
    for obj in objects:
        color = LABEL_TO_COLOR.get(obj[4])
        confidence = obj[5]
        if color in ['red', 'green'] and confidence > 0.5:
            other_objects.append((obj, color))

    model_detector.process_kalman_multi(img, brown_bear, brown_tracker, 'brown', Ts, center, kalman_coords)
    model_detector.process_kalman_multi(img, white_bear, white_tracker, 'white', Ts, center, kalman_coords)
    model_detector.process_kalman_multi(img, blue_bear, blue_tracker, 'blue', Ts, center, kalman_coords)
    model_detector.draw_other_objects(img, other_objects, center)

    return center, objects

# ======================== 初始化 ========================
LED(4).on()
time.sleep_ms(500)
LED(4).off()

uart = UART(UART_PORT, baudrate=UART_BAUDRATE)
time.sleep_ms(100)

sensor.reset()
sensor.set_pixformat(CAMERA_PIXFORMAT)
sensor.set_framesize(CAMERA_FRAMESIZE)
sensor.set_framerate(CAMERA_FRAMERATE)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_brightness(CAMERA_BRIGHTNESS)
sensor.set_contrast(2)
sensor.skip_frames(time=200)
clock = time.clock()

lcd = seekfree.IPS200(2)
lcd.full()

# 创建各模块实例
communicator = Communicator(uart)
brown_tracker = MultiTracker()
white_tracker = MultiTracker()
blue_tracker = MultiTracker()
model_detector = ModelDetector(net)
color_detector = ColorDetector()
tag_corrector = CoordinateCorrection()

# 复用列表，避免每帧新建
center = []
brown_blobs = []
white_blobs = []
blue_blobs = []
other_blobs = []

# ======================== 主循环 ========================
while True:
    clock.tick()
    img = sensor.snapshot()

    current_time = time.ticks_ms()
    delta_time = time.ticks_diff(current_time, last_time)
    Ts = max(delta_time / 1000.0, 0.01)
    last_time = current_time

    handle_uart_commands(uart)

    # 等待模式
    if current_mode == MODE_WAITING:
        gc.collect()
        continue

    # 色块模式：检测色块→多目标卡尔曼→取最下方发送
    elif current_mode == MODE_COLOR:
        all_blobs_with_color = color_detector.detect_colors(img, current_obj)
        filtered_blobs_with_color = color_detector.filter_all_blobs(all_blobs_with_color)

        center.clear()

        # 分离棕/白/蓝色块（多目标卡尔曼跟踪）与其他色块
        brown_blobs.clear()
        white_blobs.clear()
        blue_blobs.clear()
        other_blobs.clear()
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

        color_detector.process_kalman_multi(img, brown_blobs, brown_tracker, 'brown', Ts, center, kalman_coords)
        color_detector.process_kalman_multi(img, white_blobs, white_tracker, 'white', Ts, center, kalman_coords)
        color_detector.process_kalman_multi(img, blue_blobs, blue_tracker, 'blue', Ts, center, kalman_coords)
        color_detector.draw_other_blobs(img, other_blobs, center)

        # 发送离基准点(80,85)最近的目标坐标
        if center:
            dx, dy = DATUM_POINT
            target = min(center, key=lambda c: (c[0] - dx) ** 2 + (c[1] - dy) ** 2)
            communicator.send_coordinate(target[0], target[1], target[2])

    # 模型模式：YOLO检测→多目标卡尔曼→发送单个目标
    elif current_mode == MODE_MODEL:
        center, _ = detect_all_objects(img, Ts)
        is_sent = False

        if center:
            dx, dy = DATUM_POINT
            matched = [c for c in center if c[2] == current_obj]
            if matched:
                target = min(matched, key=lambda c: (c[0] - dx) ** 2 + (c[1] - dy) ** 2)
            else:
                target = min(center, key=lambda c: (c[0] - dx) ** 2 + (c[1] - dy) ** 2)
            communicator.send_coordinate(target[0], target[1], target[2])
            is_sent = True

        displayed_text = 'YES' if is_sent else 'NO'
        displayed_text_color = DRAW_COLORS['green'] if is_sent else DRAW_COLORS['red']
        img.draw_string(5, 5, displayed_text, color=displayed_text_color, scale=2)

    # 矫正模式：检测Apriltag并发送偏转角
    elif current_mode == MODE_CORRECTION:
        tag_center = tag_corrector.coordinate_correction(img)
        if tag_center is not None:
            tag_cx, tag_cy, rotation = tag_center
            rotation = (180 * rotation) / math.pi + 90
            communicator.send_coordinate_with_angle(tag_cx, tag_cy, rotation)

    # 显示图像到LCD
    lcd.show_image(img, SCREEN_WIDTH, SCREEN_HEIGHT, zoom=0)

    # 色块/模型模式下定期回收内存
    if current_mode in (MODE_COLOR, MODE_MODEL):
        frame_count += 1
        if frame_count % GC_INTERVAL == 0:
            gc.collect()
