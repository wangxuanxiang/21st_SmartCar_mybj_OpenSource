import sensor, image, time, math, mjpeg
from pyb import LED
from machine import UART
from ulab import numpy as np
import seekfree
import ustruct

##########################串口初始化#########################
uart = UART(2, baudrate=460800)
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
# LED(4).on

#####################LCD初始化#########################
lcd = seekfree.IPS200(3)
lcd.full()

######################最小变化阈值滤波#######################
position_threshold = 4 # 位置变化的最小阈值 值越小位置识别更新的越频繁，值越大小球的细微运动越不会更新识别
MAX_CHANGE_THRESHOLD = 80 # 最大变化阈值（位置和半径超过此值时不更新）
prev_x, prev_y = None, None # 上一帧的矩形中心坐标

#######################卡尔曼滤波配置##########################
last_time = time.ticks_ms()
# 观测矩阵 C，描述从状态到观测值的映射关系 C 是观测矩阵，它将状态向量（位置、速度）与观测量（图像中的矩形框信息）联系起来。这里假设观测量是位置和速度。
C = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],
[0,0,0,0,1,0],[0,0,0,0,0,1]])
"""
# 过程噪声协方差矩阵 Q，用于描述过程的随机噪声
Q_value = [
    10,   # x 位置的过程噪声（像素²）
    10,   # y 位置
    5,    # w 宽度
    5,    # h 高度
    100,  # dx 速度（像素²/s²）
    100   # dy 速度
]
Q = np.diag(Q_value) # 更新过程噪声协方差矩阵
"""
# 观测噪声协方差矩阵 R 是观测噪声协方差矩阵，表示观测过程中测量误差的大小。
R_value = [
    8,   # x 测量噪声
    8,   # y
    5,    # w
    5,    # h
    80,   # dx
    80    # dy
]
R = np.diag(R_value)
# 定义观测量Z
x = 0 # 左顶点x坐标
y = 0 # 左顶点y坐标
last_frame_x = x # 上一帧左顶点x坐标
last_frame_y = y # 上一帧左顶点y坐标
w = 0 # 矩形框宽度w
h = 0 # 矩形框高度h
dx = 0 # 左顶点x坐标移动速度
dy = 0 # 左顶点y坐标移动速度
Z = np.array([x, y, w, h, dx, dy])
# 初始状态估计
x_hat = np.array([80, 60, 30, 30, 2, 2]) #初始估计的状态值（位置，速度）
x_hat_minus = np.array([0,0,0,0,0,0]) # 初始预测的状态值
p_value = [100.0, 100.0, 50.0, 50.0, 300.0, 300.0] # 状态误差的初始值 p 是状态误差的初始协方差矩阵。
p = np.diag(p_value)
# 丢失计数器
lost_count = 0
MAX_LOST_FRAMES = 30

# 定义卡尔曼滤波函数
# 预测阶段：利用状态转移矩阵和上一状态估计预测当前状态。
# 校正阶段：通过卡尔曼增益对预测状态进行校正，使得估计值接近真实值。
# 输入 Z：观测值（或测量值），通常是来自外部传感器（例如相机、雷达等）的数据。在这个代码中，Z 是
    #一个包含目标的位置信息（如矩形框的四个角坐标）的向量，格式为 [x, y, w, h, dx, dy]，其中 x 和
    #y 是目标的中心位置，w 和 h 是目标的宽度和高度，dx 和 dy 是目标的速度。
# 输出 x_hat：更新后的状态估计，包括位置（x, y）、宽度（w, h）、速度（dx, dy）。该值是通过卡尔曼滤波器的预测和校正步骤计算得到的最优估计。
def Kalman_Filter(Z, Ts, is_detected):
    global C, Q, R, p, x_hat, x_hat_minus, lost_count
    # 动态调整Q
    if is_detected:
        Q_value = [2.0, 5.0, 2.0, 2.0, 50.0, 50.0]
        damping = 0.92
        lost_count = 0
    else:
        Q_value = [20.0, 30.0, 10.0, 10.0, 40.0, 40.0]
        damping = 0.8
        lost_count += 1
    Q = np.diag(Q_value)

    A = np.array([
        [1, 0, 0, 0, Ts, 0],
        [0, 1, 0, 0, 0, Ts],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, damping, 0],
        [0, 0, 0, 0, 0, damping]
    ])
    # 预测部分
    x_hat_minus = np.dot(A,x_hat)
    p_minus = np.dot(A,np.dot(p,A.T)) + Q

    if is_detected:
        # 校正部分
        S = np.dot(np.dot(C,p_minus),C.T) + R
        # 选择一个小的正则化项
        regularization_term = 1e-4
        # 正则化S矩阵
        S_regularized = S + regularization_term * np.eye(S.shape[0])
        # 计算正则化后的S矩阵的逆
        S_inv = np.linalg.inv(S_regularized)
        # 计算卡尔曼增益
        K = np.dot(np.dot(p_minus,C.T),S_inv)
        x_hat = x_hat_minus + np.dot(K,(Z - np.dot(C,x_hat_minus)))
        p = np.dot((np.eye(6) - np.dot(K,C)),p_minus)
    else:
        # 无观测：只预测，不校正
        x_hat = x_hat_minus
        p = p_minus

    return x_hat


########################变量定义##########################

# 目标识别中四种主要颜色的阈值
RED_THRESHOLD   = [#(0, 57, 27, 127, 7, 127),
                   #(0, 56, 9, 85, -2, 53),
                   (5, 24, 12, 41, -5, 37),
                   (30, 58, 39, 83, 10, 51)]# 红
GREEN_THRESHOLD = [#(32, 100, -128, -12, -128, 127),
                   #(42, 100, -128, -19, -128, 127),
                   (17, 67, -33, -15, -15, 68),
                   (53, 100, -51, -15, -20, 95)]# 绿(后两组暗，亮)
BLUE_THRESHOLD  = [#(34, 64, -18, 10, -128, -39),
                   #(30, 100, -30, -5, -48, -9),
                   (13, 35, -24, -9, -18, -7),
                   (37, 77, -31, -4, -54, -26)]# 蓝
BROWN_THRESHOLD = [#(32, 100, -11, 12, -16, 127),
                   #(0, 100, -128, 23, -7, 127),
                   (12, 43, -14, 14, 8, 46),
                   (51, 92, -23, 20, -16, 70)]# 棕

# 边界识别中黄色赛道的阈值
YELLOW_THRESHOLD = (70, 100, -128, 127, 10, 127)


# 目标识别感兴趣的区域
roi = (10, 0, 160, 100)

# 录像
"""
red = LED(1)
green = LED(2)
#视频文件地址
m = mjpeg.Mjpeg("/sd/example.mjpeg")
#记录视频有多少帧
fps_count = 0;
"""

# 坐标距离阈值（两个框的中心距离小于此值，就认为是“过近”，只保留一个）
DISTANCE_THRESHOLD = 30  # 可根据实际调整

# 用于标记是否已经首次检测到小熊
first_brown_detected = False

# 玩具熊最小持续帧数
brown_visible_frames = 0
last_brown_area = 0

# 模式切换
MODE_TARGET = 0
MODE_BOUNDARY_UD = 1 # U
MODE_BOUNDARY_LR = 2 # L
MODE_WAITING = 3
current_mode = MODE_WAITING

# 上一次发送的坐标
last_sent_x = 80
last_sent_y = 60


##########################目标识别函数定义##########################
# 计算两个坐标的距离
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# 分别查找各颜色色块
def detect_colors(img):
    brown_blobs   = img.find_blobs(BROWN_THRESHOLD,
    pixels_threshold=200, area_threshold=200, merge=True)
    red_blobs   = img.find_blobs(RED_THRESHOLD,
    pixels_threshold=30, area_threshold=30, merge=False)
    green_blobs   = img.find_blobs(GREEN_THRESHOLD,
    pixels_threshold=40, area_threshold=40, merge=False)
    blue_blobs   = img.find_blobs(BLUE_THRESHOLD,
    pixels_threshold=30, area_threshold=30, merge=False)

    all_blobs_with_color = []
    for blob in brown_blobs:
        all_blobs_with_color.append((blob, 'brown'))
    for blob in red_blobs:
        all_blobs_with_color.append((blob, 'red'))
    for blob in green_blobs:
        all_blobs_with_color.append((blob, 'green'))
    for blob in blue_blobs:
        all_blobs_with_color.append((blob, 'blue'))
    return all_blobs_with_color

# 对所有色块去重
def filter_all_blobs(blobs):
    filtered = []
    for item in blobs:
        blob = item[0]
        color = item[1]

        # 密度滤波 + 动态面积阈值
        if blob.density() < 0.3:
            continue
        min_pixels = 50 * (blob.density() + 0.5)
        if blob.pixels() < min_pixels:
            continue

        # 过滤宽度过大的色块
        if (
            # 长大于120，宽大于100，直接舍弃
            blob.w() > 140
            or blob.h() > 110

            # 棕色规则：边长比超过3:1
            or (
                color == 'brown'
                and ((blob.w() > 3 * blob.h() or blob.h() > 3 * blob.w()))
            )

            # 绿/蓝规则：边长比超过1.5:1
            or (
                color in ('green', 'blue')
                and (blob.w() > 1.5 * blob.h() or blob.h() > 1.5 * blob.w() or abs(blob.w() - blob.h()) > 10)
            )
        ):
            continue
        cx, cy = blob.cx(), blob.cy()
        keep = True
        # 对比已保留的色块，判断距离是否过近
        for saved_item in filtered:
            saved_blob = saved_item[0]
            distance = calculate_distance(cx, cy, saved_blob.cx(), saved_blob.cy())
            if distance < DISTANCE_THRESHOLD:
                keep = False
                break
        if keep:
            filtered.append(item)
    return filtered

# 判断玩具熊有效性
def is_brown_valid(blob):
    global last_brown_area, brown_visible_frames
    current_area = blob.area()
    if last_brown_area > 0:
        area_change_rate = abs(current_area - last_brown_area) / last_brown_area
        if area_change_rate > 0.7:
            return False
    last_brown_area = current_area
    brown_visible_frames += 1
    return True

# 目标识别串口发送函数
def send_coordinate(x, y):
    global uart, last_sent_x, last_sent_y
    dx_coord = min(30, max(-30, x - last_sent_x))
    dy_coord = min(30, max(-30, y - last_sent_y))
    x_limited = last_sent_x + dx_coord
    y_limited = last_sent_y + dy_coord
    last_sent_x = x_limited
    last_sent_y = y_limited
    data = ustruct.pack("<BBBBB",
                        0xA5,
                        0xA6,
                        x_limited,
                        y_limited,
                        0x5B
                        )
    uart.write(data)
    # print(x,y)

#######################边界识别函数定义#######################

# 边界识别串口发送函数
def send_angle(angle):
    global uart
    angle_mapped = angle + 90
    data = ustruct.pack("<BBBB",
                        0xA5,
                        0xA7,
                        angle_mapped,
                        0x5B
                        )
    uart.write(data)
# 边界识别
def boundary_correction(mode, img):
    blobs = []
    center = 0

    if mode == 'row': #行
        num = [0, 26, 52, 80, 106, 132]
    elif mode == 'column': #列
        num = [0, 20, 40, 60, 80, 100]
    for x in num:
        if mode == 'row': # 从左到右找色块
            result = img.find_blobs([YELLOW_THRESHOLD], roi = [x,0,26,120] ,pixels_threshold=100, area_threshold=100, margin=1, merge=True, invert=0)
        elif mode == 'column':# 从上到下找色块
            result = img.find_blobs([YELLOW_THRESHOLD], roi = [0,x,160,20] ,pixels_threshold=100, area_threshold=100, margin=1, merge=True, invert=0)
        if result:
            best_blob = min(result, key= lambda b: abs(b.area() - 600))
            blobs.append(best_blob)
            center += 1
            img.draw_rectangle(best_blob.rect(), color = (255, 0, 0), scale = 1, thickness = 1)

    if center >= 3:
        l = img.get_regression([YELLOW_THRESHOLD])
        if l:
            img.draw_line(l.line(), color = (255, 0, 0), thickness = 2)
            theta = l.theta()
            if theta > 90:
                angle = -(theta - 180)
            else:
                angle = -theta
        return angle
    else:
        return None
        # else:
            # print("no found") # 调试用
    # else:
        # print("insufficient blobs") # 调试用

########################指令解析函数定义###########################
def handle_uart_commands():
    global current_mode
    if not uart.any():
        return

    cmd = uart.read(1)

    if cmd == b'T':
        current_mode = MODE_TARGET
    elif cmd == b'U':
        current_mode = MODE_BOUNDARY_UD
    elif cmd == b'L':
        current_mode = MODE_BOUNDARY_LR
    elif cmd == b'F':
        current_mode = MODE_WAITING

    return


############################主部分###########################
while(True):
    clock.tick()
    img = sensor.snapshot()
    # white.on() # 可以补光
    # white.off()
    current_time = time.ticks_ms()
    delta_time = time.ticks_diff(current_time,last_time)
    Ts = max(delta_time / 1000.0, 0.01)
    last_time = current_time

    # 录像
    # red.on()
    """
    #如果帧数没达到1000
    if fps_count < 1000:
        #保存当前图片为1帧
        m.add_frame(img)
        print(clock.fps())
        fps_count += 1
    else:
        #关闭文件才保存成功，需要传入保存视频的帧率，可以自己设定，参数填24表示保存的视频就是1秒钟播放24帧
        m.close(60)
        red.off()
        green.on()
        time.sleep_ms(500)
        break
    """
    handle_uart_commands()
    if current_mode == MODE_WAITING:
        LED(1).on()
        LED(1).off()
        continue
    elif current_mode == MODE_TARGET:
        LED(2).on()
        LED(2).off()
        # print("1")
        # 获取图像并进行预处理
        all_blobs_with_color = detect_colors(img)
        filtered_blobs_with_color = filter_all_blobs(all_blobs_with_color)

        # 分离棕色与其它
        brown_blobs = []
        other_blobs = []

        for item in filtered_blobs_with_color:
            blob = item[0]
            color = item[1]
            if color == 'brown':
                brown_blobs.append(blob)
            else:
                other_blobs.append((blob, color))

        # 绘制筛选后的色块（不同颜色用不同框区分）
        draw_colors = {
            'red': (255, 0, 0),
            'green': (0, 255, 0),
            'blue': (0, 0, 255),
            'brown': (255, 255, 255),
            'grey': (100, 100, 100)
        }

        center = []

        # 处理棕色
        brown_detected = False
        if brown_blobs:
            target_brown = max(brown_blobs, key = lambda b:b.area())
            if is_brown_valid(target_brown):
                x, y, w, h = target_brown.rect()
                max_speed = 80
                dx_raw = (x - last_frame_x) / Ts
                dx = max(-max_speed, min(max_speed, dx_raw))
                dy_raw = (y - last_frame_y) / Ts
                dy = max(-max_speed, min(max_speed, dy_raw))
                Z = np.array([x, y, w, h, dx, dy], dtype = np.float)
                # 如果是第一次检测到小熊，则初始化卡尔曼滤波器
                if not first_brown_detected:
                    # 初始化卡尔曼滤波器的状态和协方差矩阵
                    last_frame_x, last_frame_y = x, y
                    x_hat = np.array([x, y, w, h, 0, 0])  # 初始估计的状态值（位置，速度）
                    p_value = [10.0, 10.0, 5.0, 5.0, 100.0, 100.0]  # 状态误差的初始值
                    p = np.diag(p_value)
                    first_brown_detected = True  # 标记为已首次检测过
                x_hat = Kalman_Filter(Z, Ts, is_detected=True)
                last_frame_x, last_frame_y = x, y
                brown_detected = True
                img.draw_rectangle(target_brown.rect(), color=draw_colors['brown'])  # 画矩形框
                img.draw_cross(target_brown.cx(), target_brown.cy(), color=draw_colors['brown'])  # 画中心点
        else:
            brown_visible_frames = 0
        # 无检测时预测
        if not brown_detected and first_brown_detected:
            if lost_count < MAX_LOST_FRAMES:
                x_hat = Kalman_Filter(None, Ts, is_detected=False)

        # 绘制卡尔曼预测框（灰色）
        if first_brown_detected and lost_count < MAX_LOST_FRAMES:
            kx, ky = int(x_hat[0]), int(x_hat[1])
            kw, kh = max(1, int(x_hat[2])), max(1, int(x_hat[3]))
            kcx, kcy = kx + kw // 2, ky + kh // 2

            img.draw_rectangle(kx, ky, kw, kh, color=draw_colors['grey'])
            img.draw_cross(kcx, kcy, color=draw_colors['grey'])
            center.append((kcx, kcy))
        if lost_count >= MAX_LOST_FRAMES:
            first_brown_detected = False
            lost_count = 0
        for item in other_blobs:
            # 绘制该颜色的所有筛选后色块
            blob = item[0]
            color_name = item[1]
            img.draw_rectangle(blob.rect(), color=draw_colors[color_name])  # 画矩形框
            img.draw_cross(blob.cx(), blob.cy(), color=draw_colors[color_name])  # 画中心点
            center_x = blob.cx()
            center_y = blob.cy()
            center.append((center_x, center_y))
        if center:
            target = max(center, key = lambda coordinate : coordinate[1]) # 选择最靠近小车的坐标（判断依据为y最大的坐标）
            target_x, target_y = target
            send_coordinate(target_x, target_y)
    elif current_mode == MODE_BOUNDARY_UD:
        LED(3).on()
        LED(3).off()
        # print("2")
        angle = boundary_correction('row', img)
        if angle:
            send_angle(angle)
    elif current_mode == MODE_BOUNDARY_LR:
        LED(4).on()
        LED(4).off()
        # print("3")
        angle = boundary_correction('column', img)
        if angle:
            send_angle(angle)


    lcd.show_image(img, 160, 120, zoom=0) # 外接LCD屏幕
    print(f"FPS: {clock.fps()}")
