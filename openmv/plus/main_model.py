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

# 去噪配置
MIN_DETECT_AREA = 10   # 最小检测面积（像素），低于此视为噪点

# 通信协议常量
PROTOCOL_HEADER1 = 0xA5
PROTOCOL_HEADER2_COORD = 0xA6
PROTOCOL_HEADER2_ANGLE = 0xA7
PROTOCOL_HEADER2_APRILTAG = 0xA8
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

# YOLO模型路径（基于OpenMV官方YOLO模型训练，支持5类物体检测）
face_detect = '/sd/yolo3_iou_smartcar_final_with_post_processing.tflite'
# 载入模型
net = tf.load(face_detect)

# 帧计数器及时间戳（用于调试和性能统计）
global_counter = 0
first_time = time.ticks_ms()

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

    def send_coordinate_with_angle(self, tag_cx, tag_cy, rotation):
        """发送Apriltag坐标及偏转角度

        Args:
            tag_cx: Apriltag中心x坐标（像素）
            tag_cy: Apriltag中心y坐标（像素）
            rotation: 偏转角度（弧度），内部转换为角度×10的整数格式
        """
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

    负责调用TFLite模型进行推理，管理brown/white物体的卡尔曼跟踪，
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

# ======================== 坐标矫正模块 ========================
class CoordinateCorrection:
    """Apriltag检测与坐标校正

    检测TAG25H9家族的Apriltag，计算相机相对于Tag的偏转角度，
    用于机器人坐标系的视觉校正。
    """

    def __init__(self):
        self.tag_family = image.TAG25H9

    def coordinate_correction(self, img):
        """检测Apriltag并计算偏转角度"""
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

# 运行模式定义（通过UART命令切换）
MODE_CORRECTION = 0      # 坐标校正：检测Apriltag并发送角度
MODE_MODEL = 1           # 模型模式：检测物体→卡尔曼跟踪→发送单个目标
MODE_PREVIEW = 2         # 预览模式：检测所有物体→打包发送（上位机完整展示）
MODE_WAITING = 3         # 等待模式：空闲，仅维持摄像头画面显示
current_mode = MODE_WAITING

# 各颜色对应的卡尔曼预测坐标（供TargetLocker使用），默认值为屏幕中心
kalman_coords = {
    'brown': (SCREEN_CENTER_X, SCREEN_CENTER_Y),
    'white': (SCREEN_CENTER_X, SCREEN_CENTER_Y)
}

# 上一帧的时间戳，用于计算卡尔曼滤波的时间步长Ts
last_time = time.ticks_ms()

# ======================== 工具函数 ========================
# 当前选中目标类型（由下位机通过UART指定）
current_obj = ''

def handle_uart_commands(uart):
    """处理来自下位机的串口命令：切换运行模式 OR 更新当前目标物体类型

    模式切换命令:
        C → MODE_CORRECTION (坐标校正)
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

        # ---------- 模式切换命令 ----------
        if cmd == b'C':
            current_mode = MODE_CORRECTION
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

        # ---------- 物体类型命令 ----------
        elif cmd == b'b':
            current_obj = 'brown'
        elif cmd == b'w':
            current_obj = 'white'
        elif cmd == b's':
            current_obj = 'red'
        elif cmd == b'e':
            current_obj = 'blue'
        elif cmd == b't':
            current_obj = 'green'


# 当前选中目标类型（由下位机通过UART指定）
current_obj = ''
if_change_mode = False


def detect_all_objects(img, Ts):
    """执行全量目标检测，按颜色分类处理并汇总结果

    处理流程:
        1. YOLO模型推理，获取所有检测框
        2. brown/white物体 → 卡尔曼滤波跟踪（置信度阈值0.3）
        3. red/green/blue物体 → 直接绘制检测框（置信度阈值0.5或0.3）
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

    other_objects = []
    for obj in objects:
        color = LABEL_TO_COLOR.get(obj[4])
        confidence = obj[5]
        if color in ['red', 'green'] and confidence > 0.5:
            other_objects.append((obj, color))
        elif color == 'blue' and confidence > 0.3:
            other_objects.append((obj, color))

    # brown/white使用卡尔曼跟踪（含丢失预测），其他颜色直接绘制
    model_detector.process_kalman_color(img, brown_bear, brown_tracker, 'brown', Ts, center, kalman_coords)
    model_detector.process_kalman_color(img, white_bear, white_tracker, 'white', Ts, center, kalman_coords)
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
tag_corrector = CoordinateCorrection()
communicator = Communicator(uart)
brown_tracker = KalmanTracker()
white_tracker = KalmanTracker()
model_detector = ModelDetector(net)

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

    # 坐标校正模式：检测Apriltag并发送偏转角度
    elif current_mode == MODE_CORRECTION:
        tag_center = tag_corrector.coordinate_correction(img)
        if tag_center is not None:
            tag_cx, tag_cy, rotation = tag_center
            rotation = (180 * rotation) / math.pi + 90
            communicator.send_coordinate_with_angle(tag_cx, tag_cy, rotation)

    # 模型模式：执行目标检测与跟踪，选择一个目标发送坐标
    elif current_mode == MODE_MODEL:
        center, objects = detect_all_objects(img, Ts)
        is_sent = False
        """
        if current_obj == 'red':
            LED(4).on()  # 红色沙包时点亮LED4
        else:
            LED(4).off()  # 非红色沙包时关闭LED4
        """
        # 目标选择策略：优先匹配已选类型的物体（取最下方），否则取所有物体中最下方的
        if center:
            matched = [c for c in center if c[2] == current_obj]
            if matched:
                target = max(matched, key=lambda c: c[1])
            else:
                target = max(center, key=lambda c: c[1])
            communicator.send_coordinate(target[0], target[1], target[2])
            global_counter += 1
            is_sent = True

        displayed_text = 'YES' if is_sent else 'NO'
        displayed_text_color = DRAW_COLORS['green'] if is_sent else DRAW_COLORS['red']
        img.draw_string(5, 5, displayed_text, color=displayed_text_color, scale=2)

    # 预览模式：执行全量检测，将所有检测目标打包发送（供上位机完整展示）
    elif current_mode == MODE_PREVIEW:
        center, objects = detect_all_objects(img, Ts)
        communicator.pack_center_data(center)

    # 显示图像到LCD
    lcd.show_image(img, SCREEN_WIDTH, SCREEN_HEIGHT, zoom=0)