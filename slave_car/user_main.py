# 包含 gc 与 time 类
import gc
import time
import os
from micropython import const
gc.collect()
# 从 machine 库包含所有内容 
from machine import *
gc.collect()
from display import *
gc.collect()
from seekfree import MOTOR_CONTROLLER, IMU660RX, KEY_HANDLER, BLDC_CONTROLLER
gc.collect()
from smartcar import ticker, encoder
my_uart3 = UART(2)
my_uart3.init(115200)
my_uart2 = UART(7)
my_uart2.init(115200)
import ant_plan
gc.collect()
import ant_else
gc.collect()
# 与定时器2周期一致，都为53ms
pin_obj = Pin("C15", Pin.OPEN_DRAIN, pull = Pin.PULL_UP, value = True)
del(pin_obj)
pin_obj = Pin("C15", Pin.OPEN_DRAIN, pull = Pin.PULL_UP, value = True)
if_menu = False
if not pin_obj.value():#进入调试模式
    if_menu = True
    import ant_menu
    gc.collect()
else:
    gc.collect()
    import ant_move
    gc.collect()
    import ant_task
    gc.collect()
    import ant_vision
    gc.collect()
    import ant_motor
    gc.collect()
    # 引入 VL53L4CD 驱动
    from vl53l4cd import VL53L4CD
    import os

###################################【变量定义及初始化】###################################
PI = const(3.1415926)
READY_NAVIGATE = const(0) # 准备导航状态
NAVIGATE = const(1)       # 导航状态
SCAN = const(2)           # 扫描状态
SERVO = const(3)          # 视觉伺服状态
ORBIT = const(4)          # 环绕状态
MOVE = const(5)           # 搬运状态
CALIBRATE = const(6)      # 校准状态
ADJUST = const(7)         # 微调状态
RETURN = const(8)		  # 返回状态
STOP = const(9)           # 停止状态
RETREAT = const(10)       # 后退状态

# 多路复用时间计数器
counter = 0      # type: int
# 是否按下启动按键标志位
if_press_start_key = False
# 是否成功启动标志位
start_flag = False

##################################【实例对象构建及初始化】##################################
"""""""""核心板与学习板接口初始化"""""""""
# 核心板上 C4 是 LED
# 学习板上 D9  对应一号拨码开关
led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# 构造输入电压分压检测电路接口
power_adc = ADC('B27')

# 定时器初始化
pit1 = ticker(1)
pit2 = ticker(2)
pit3 = ticker(3)

"""蜂鸣器初始化"""
beep = Pin('D24', Pin.OUT, value = False)
# 创建蜂鸣器对象
my_beep = ant_else.beep(beep)

"""异步串口通信初始化"""
my_uart6 = UART(5)
my_uart6.init(115200)

"""无线串口通信初始化"""
my_uart3 = UART(2)
my_uart3.init(115200)
my_uart2 = UART(1)
my_uart2.init(115200)
os.dupterm(my_uart2)

# ------------------------------------------------------------------------------
#   初始化 I2C 总线
#   对于 RT1021-144P-BTB 核心板：
#     I2C0: B30(SCL) / B31(SDA)   |   I2C3: D22(SCL) / D23(SDA)
#   注意：BTB 核心板的 LPI2C3 (id=2) 不能使用，请避开 id=2。
# ------------------------------------------------------------------------------
i2c_1 = I2C(1, freq = 100000)
i2c_3 = I2C(3, freq = 100000)

# 扫描 I2C 总线确认设备在线
# ------------------------------------------------------------------------------
#   device_list = I2C.scan()
#       return          返回内容    |   返回从 0x08 到 0x77 地址有响应的从机地址列表 字节数组形式
# ------------------------------------------------------------------------------
device_list_0 = i2c_1.scan()
device_list_3 = i2c_3.scan()
tof_L = None
tof_R = None

# 初始化传感器 1 (I2C1)
if 0x29 in device_list_0:
    try:
        tof_R = VL53L4CD(i2c_1, address = 0x29)
    except Exception as e:
        my_beep.failure_to_find_tof()
else:
    my_beep.failure_to_find_tof()
    print("TOF R (I2C1) not found! Please check wiring and XSHUT pull-up.")

# 初始化传感器 3 (I2C3)
if 0x29 in device_list_3:
    try:
        tof_L = VL53L4CD(i2c_3, address = 0x29)
    except Exception as e:
        my_beep.failure_to_find_tof()
        print("TOF L (I2C3) init failed:", e)
else:
    my_beep.failure_to_find_tof()
    print("TOF L (I2C3) not found! Please check wiring and XSHUT pull-up.")


"""光电管初始化"""
photo = Pin('B4', Pin.IN, value = False)

"""电机初始化"""
motor_ul = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = True)
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = True)
motor_md = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5  , 13000, duty = 0, invert = True)

"""传感器初始化"""
# 编码器初始化
encoder_ul = encoder("C2" , "C3" , True)
encoder_ur = encoder("D13", "D14", True)
encoder_md = encoder("D16", "D15", True)

# IMU初始化
imu = IMU660RX()

"""菜单与显示屏初始化"""
# 新建LCD实例并初始化
cs = Pin('B29' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
cs.high()
cs.low()
rst = Pin('B31' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
dc  = Pin('B5' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
blk = Pin('C21' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
drv = LCD_Drv(SPI_INDEX = 2, BAUDRATE = 60000000, DC_PIN = dc, RST_PIN = rst, LCD_TYPE = LCD_Drv.LCD200_TYPE)
lcd = LCD(drv)
lcd.color(0xFFFF, 0x0000)
lcd.mode(0)
lcd.clear(0x0000)

key = KEY_HANDLER(53)
key_data = key.get()
# 按键对应的数据接口
"""
key_up:     key_data[1]
key_down:   key_data[0]
enc_key:    key_data[2]
key_run:    key_data[3] 
"""

# 菜单编码器初始化
enc_rotation = encoder("C0", "C1", True)

"""""""""创建对象"""""""""
# 创建状态机对象
my_state = ant_plan.StateMachine()

fan = BLDC_CONTROLLER(BLDC_CONTROLLER.PWM_C25, freq=300, highlevel_us = 1000)

#【文件读取】
# 从main_config.txt中读取保存所有的参数并保存到config字典中
my_flash_sys = ant_else.flash_system(my_beep, "/flash/slave_config.txt")
my_flash_sys.phase_config()
# 检查列表格式
my_flash_sys.check_list_format()

# 创建指令管理对象
my_order_manager = ant_else.order_manager(my_flash_sys, my_uart6)

# 创建openart串口解析对象
my_art_protocol = ant_else.UARTProtocol(my_uart6)

# 创建主从车无线串口通信对象
my_slave_protocol = ant_else.LinkProtocol(my_uart3)

# 创建pid参数对象
pid_data = ant_motor.PID_data(my_flash_sys)

#创建无刷
my_fan = ant_motor.FanControl(my_flash_sys, fan, my_state)

# 创建光电管控制对象
my_photo = ant_motor.PhotoControl(my_flash_sys, my_beep, photo)

# 创建电机微分项的滑动平均滤波器对象
diff_filter_ul = ant_motor.SlipAveragingFilter(3)    # 滤波窗口为2个
diff_filter_ur = ant_motor.SlipAveragingFilter(3)    # 滤波窗口为3个
diff_filter_md = ant_motor.SlipAveragingFilter(5)    # 滤波窗口为2个
diff_filter_gyroz = ant_motor.SlipAveragingFilter(3)  # 滤波窗口为5个

# 创建加速度计滤波对象
acc_x_fil = ant_motor.SlipAveragingFilter(5)
acc_y_fil = ant_motor.SlipAveragingFilter(5)
acc_z_fil = ant_motor.SlipAveragingFilter(5)
acc_z_fil.buffer_init(4096)  # 初始化z轴加速度计滤波器的初始值为4096

# 创建小车x和y方向上的速度的卡尔曼滤波器
speed_x_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
speed_y_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
# 创建小车自转角滤波器对象
car_yaw_fil = ant_motor.SlipAveragingFilter(1)
# 创建视觉伺服正余弦滤波对象
sin_servo_fil = ant_motor.SlipAveragingFilter(4)    
cos_servo_fil = ant_motor.SlipAveragingFilter(4)
# 创建环绕控制航向角/转角滤波对象（视觉模式下滑动平均，平滑视觉噪声）
orbit_yaw_fil = ant_motor.SlipAveragingFilter(3)
orbit_turn_angle_fil = ant_motor.SlipAveragingFilter(3)
# 创建距离控制滤波对象
dist_fil_L = ant_motor.SlipAveragingFilter(3)
dist_fil_R = ant_motor.SlipAveragingFilter(3)

# 创建姿态数据对象
pose_data = ant_motor.PoseData(my_flash_sys, my_uart3, imu, encoder_ul, encoder_ur, encoder_md, diff_filter_gyroz, acc_x_fil, acc_y_fil, acc_z_fil)

# 创建电机pid对象和角度pid对象
motor_ul_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ul)
motor_ur_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ur)
motor_md_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_md)
angle_pid = ant_motor.AnglePositionPID(my_flash_sys)
servo_pid = ant_motor.ServoPID(my_flash_sys)
dist_pid_L = ant_motor.DistPID(my_flash_sys, "L", dist_fil_L)
dist_pid_R = ant_motor.DistPID(my_flash_sys, "R", dist_fil_R)

# 创建小车姿态对象
my_car = ant_motor.CarPose(my_flash_sys, my_state, pose_data, car_yaw_fil, angle_pid,
                        motor_ul_pid, motor_ur_pid, motor_md_pid,
                        motor_ul, motor_ur, motor_md)

# 创建路径规划数据对象
plan_data = ant_plan.PlanData(my_flash_sys)

# 创建路径规划对象
my_path = ant_plan.PathPlan(plan_data, my_car)
# 创建规划（路径和速度）对象
my_plan = ant_plan.NavigationPlan(my_flash_sys,my_fan, plan_data, my_car, my_state, my_order_manager, my_uart3, my_beep, my_art_protocol, angle_pid)

my_tof = ant_else.TofControl(my_flash_sys, my_beep, my_car, my_plan, dist_pid_L, dist_pid_R, tof_L, tof_R)

# 创建视觉伺服管理对象2
my_vision_manager = ant_vision.VisionManager(my_flash_sys, my_beep, pose_data, angle_pid, servo_pid, sin_servo_fil, cos_servo_fil, orbit_yaw_fil, orbit_turn_angle_fil, my_uart3, my_car, my_art_protocol, my_order_manager, my_plan, my_state)

# 搬运控制类
my_moving = ant_move.MoveControl(my_flash_sys,my_beep, my_photo, my_uart3, my_uart2, my_car, my_plan, my_path, plan_data, my_vision_manager, my_state, my_slave_protocol, my_art_protocol, my_order_manager, my_tof, angle_pid)
# 任务及类
my_task = ant_task.TaskController(my_flash_sys,my_beep, my_state, my_uart3, my_car, my_path, my_plan, my_vision_manager,  my_moving, plan_data, my_order_manager, my_art_protocol,  my_slave_protocol, my_tof, angle_pid)

motor_control_T = my_flash_sys.find_value("motor_control_T")
uart_and_menu_T = my_flash_sys.find_value("uart_and_menu_T")
plan_calculate_T = my_flash_sys.find_value("plan_calculate_T")

my_flash_sys.release_config()  # 释放配置文件占用的内存
# 测试打印变量解析是否成功
"""
print("fixed+point:", plan_data.fixed_point)
print("center_rect:", plan_data.center_rect)
print("rectangle_obstacles:", plan_data.rectangle_obstacles)
"""

# 创建菜单对象
# my_menu = ant_menu.Menu(my_flash_sys, my_beep, lcd, enc_rotation, key_data, key)
###################################【函数定义】###################################
# 电机驱动函数
def set_motor(motor, duty) -> None:
    motor.duty(duty)

# 是否成功读取文件和开启定时器检查函数
def detect_if_normal() -> None:
    led.toggle()
    my_beep.test()

# 检测电源电压函数
def voltage_detect(limit_min: float) -> None:
    power_adc_value = power_adc.read_u16()
    power_voltage = power_adc_value / 65535 * 3.3 * 11
    print(f"The current power supply voltage is {power_voltage}!")
    if power_voltage <= limit_min:
        print(f"The power supply voltage: {power_voltage} is too low!")
        my_beep.low_power_warn()

# 角度环计算函数
def angle_pid_compute():
    # 计算z轴的目标速度
    angle_pid.compute_pid(my_car.turn_angle_target, my_car.now_yaw * 180 / PI)

# 用于从车启动的函数
def slave_start():
    global current_time, last_left_time, start_flag, if_press_start_key
    if start_flag == False:
        if if_press_start_key == False:
            if key_data[3] != 0:
                # 清除按键状态
                key.clear(4)
                my_beep.key_test()
                if_press_start_key = True#按下启动按键后等待主车发送开始信号
        else:   
            # 测试，此时只调试从车，双车正常通信时需要解注释  
            if my_slave_protocol.get_start_signal() == True:
                my_beep.test()
                my_slave_protocol.send_slave_state("ready")
                # 此时开启无刷负压风扇
                my_fan.set_fan_signal()
                # 初始状态设置为准备导航状态
                my_state.state =READY_NAVIGATE
                start_flag = True
                # 延时1秒避免零漂校准不准确
                time.sleep_ms(1000)
                # 打开定时器1和3
                pit1_start()
                pit3_start()
                # 检测是否正常初始化所有
                detect_if_normal()
                # 初始化小车坐标及偏航角
                my_car.x_current = plan_data.fixed_point[0][0]
                my_car.y_current = plan_data.fixed_point[0][1]

# 小车姿态总控制函数
def master_control():
    if my_state.state in [NAVIGATE, READY_NAVIGATE, RETURN, STOP, SCAN, RETREAT]:
        my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == MOVE:
        if my_moving.current_state == ORBIT:
            my_car.move_ctrl(my_vision_manager.orbit_speed, my_vision_manager.orbit_yaw, my_vision_manager.orbit_turn_angle)
        elif my_moving.current_state in [SERVO, ADJUST]:
            if not my_vision_manager.if_lost_object:
                my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
            else:
                my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
        elif my_moving.current_state in [NAVIGATE, SCAN]:
            my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
        elif my_moving.current_state == MOVE:
            if my_plan.fitting_path_:my_car.move_ctrl(my_plan.target_v, my_plan.fit_target_yaw, my_plan.turn_angle_target)
            else:my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state in [SERVO]:
        # 未丢失物体时正常进行视觉伺服控制，丢失物体时进行矩形轨迹的导航控制
        if my_vision_manager.if_lost_object == False:
            my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
        else:
            my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == ORBIT:
        my_car.move_ctrl(my_vision_manager.orbit_speed, my_vision_manager.orbit_yaw, my_vision_manager.orbit_turn_angle)
    elif my_state.state ==  ADJUST:
        if my_task.blind_box_state == NAVIGATE:
            my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
        elif my_task.blind_box_state == SERVO:
            my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
        elif my_task.blind_box_state == ORBIT:
            my_car.move_ctrl(my_vision_manager.orbit_speed, my_vision_manager.orbit_yaw, my_vision_manager.orbit_turn_angle)

# 根据目标速度选择对应挡位的PID参数（gain scheduling）
# 阈值常量
_HIGH_TARGET = 180         # >= 此值使用High挡
_MID_TARGET = 120          # >= 此值使用Mid→High线性插值
_LOW_TARGET = 50           # >= 此值使用Low→Mid线性插值，< 此值使用Low挡

def _select_pid_params(motor_pid, kp_high, ki_high, kd_high,
                       kp_mid, ki_mid, kd_mid,
                       kp_low, ki_low, kd_low):
    """为单个电机按目标速度选择并设置PID参数"""
    target_abs = abs(motor_pid.target)

    if target_abs >= _HIGH_TARGET:
        motor_pid.set_pid_params(kp_high, ki_high, kd_high)
    elif target_abs >= _MID_TARGET:
        ratio = (target_abs - _MID_TARGET) / (_HIGH_TARGET - _MID_TARGET)
        motor_pid.set_pid_params(
            kp_mid + (kp_high - kp_mid) * ratio,
            ki_mid + (ki_high - ki_mid) * ratio,
            kd_mid + (kd_high - kd_mid) * ratio)
    elif target_abs >= _LOW_TARGET:
        ratio = (target_abs - _LOW_TARGET) / (_MID_TARGET - _LOW_TARGET)
        motor_pid.set_pid_params(
            kp_low + (kp_mid - kp_low) * ratio,
            ki_low + (ki_mid - ki_low) * ratio,
            kd_low + (kd_mid - kd_low) * ratio)
    else:
        motor_pid.set_pid_params(kp_low, ki_low, kd_low)

def set_pid_params():
    if my_state.state == MOVE:
        motor_ul_pid.set_pid_params(pid_data.ul_high_kp, pid_data.ul_high_ki, pid_data.ul_high_kd)
        motor_ur_pid.set_pid_params(pid_data.ur_high_kp, pid_data.ur_high_ki, pid_data.ur_high_kd)
        motor_md_pid.set_pid_params(pid_data.md_high_kp, pid_data.md_high_ki, pid_data.md_high_kd)
    else:
        _select_pid_params(motor_ul_pid,
            pid_data.ul_high_kp, pid_data.ul_high_ki, pid_data.ul_high_kd,
            pid_data.ul_mid_kp, pid_data.ul_mid_ki, pid_data.ul_mid_kd,
            pid_data.ul_low_kp, pid_data.ul_low_ki, pid_data.ul_low_kd)
        _select_pid_params(motor_ur_pid,
            pid_data.ur_high_kp, pid_data.ur_high_ki, pid_data.ur_high_kd,
            pid_data.ur_mid_kp, pid_data.ur_mid_ki, pid_data.ur_mid_kd,
            pid_data.ur_low_kp, pid_data.ur_low_ki, pid_data.ur_low_kd)
        _select_pid_params(motor_md_pid,
            pid_data.md_high_kp, pid_data.md_high_ki, pid_data.md_high_kd,
            pid_data.md_mid_kp, pid_data.md_mid_ki, pid_data.md_mid_kd,
            pid_data.md_low_kp, pid_data.md_low_ki, pid_data.md_low_kd)

# 测试tof距离控制
def test_tof_distance_control():
    if my_state.state == READY_NAVIGATE:
        my_state.state = MOVE
        my_plan.move_v_max = 160
        my_moving.current_state = MOVE
        my_plan.move_state = MOVE
        my_plan.keep_x_or_y_v = False
        my_tof.ready_tof('left', 0, 'S')
        my_car.x_current = 0.0
        my_car.y_current = 0.0
    elif my_state.state == MOVE:
        # 距离控制
        my_tof.dist_control()
        my_plan.navigate(path = [[50.0, 50.0], [50.0, 150.0]], target_turn_angle = -45.0)
        if my_plan.if_finish_navigate == True:
            my_tof.reset_tof()
            my_plan.reset_navigate()
            my_plan.reset_navigate_angle()
            my_state.state = STOP
    elif my_state.state == STOP:
        pass
        # my_uart3.write(f"slave_car: {my_car.x_current},{my_car.y_current}\n")
# 任务机执行函数
def task_machine():
    my_task.run()

# 视觉伺服测试函数

""" 定时器类 """
# 定时器1中断回调函数
def time_pit1_handler(time):
    # 更新传感器数据
    pose_data.update_data()

    # 更新小车姿态
    my_car.update_pose()

    # 测试角度闭环
    #complete_angle_circle()

    # 速度环测试
    #show_speed_PID_test()
    
    # 总控制函数
    master_control()

    # 更新pid参数
    set_pid_params()

    # 设置电机pwm输出
    my_car.set_motor_pwm()

# 定时器3中断处理函数：路径规划与速度规划计算
def time_pit3_handler(time) -> None:
    # 角度环计算（10ms）
    angle_pid_compute()

    # 任务执行机
    task_machine()
    # my_plan.navigate([plan_data.fixed_point[1], plan_data.fixed_point[3], plan_data.fixed_point[2], plan_data.fixed_point[0]])
    
    # 视觉伺服测试程序
    # test_vision_servo()

    # 边线校准测试程序
    # test_apriltag_calibrate()

    # 环绕物体测试程序
    # test_orbit()

    # 测试tof距离控制
    # test_tof_distance_control()

    # 自转测试函数
    # test_spin()

    # 环绕物体测试程序
    # test_orbit()

    # apriltag码矫正测试函数
    # test_apriltag_calibrate()
    pass


# 定时器2中断回调函数
# 用于无线串口调试和发车启动
def time_pit2_handler(time):
    """用于无线串口调试"""
    # 发车启动函数
    slave_start()

    # 读取按键（中断中避免阻塞，快速返回）
    """
    key = my_menu.read_key()
    my_menu.handle_key_from_interrupt(key)
    """

    # 更新tof传感器信息
    # my_tof.update_tof()
    # print(f"{my_state.state},{my_task.blind_box_state},{my_vision 我就写没东西，没有，没有，没有，没东西，没东西，没东西。 只要你好。 圆圆圆圆。_manager.if_finish_servo},{my_vision_manager.target_point},{my_vision_manager.current_servo_object}")
    # my_uart2.write(f"{my_state.state},{my_moving.current_state},{my_plan.if_finish_navigate},{my_moving.if_finish_move}\r\n")
    # my_uart2.write(f"{my_tof.data_L},{my_tof.data_R},{dist_pid_L.pwm_output},{dist_pid_R.pwm_output},{my_car.speed_weight}\r\n")
    # my_uart2.write(f"{my_vision_manager.car_position},{my_vision_manager.rel_pos_to_apriltag},{my_car.x_current},{my_car.y_current}\r\n")
    # my_uart3.write(f"{pose_data.now_yaw}, {my_car.now_yaw * 180 / PI}\r\n")
    # my_uart3.write(f"{my_vision_manager.if_ready_calibrate},{my_vision_manager.if_gain_calibrate_angle},{my_vision_manager.calibrate_times},{my_vision_manager.target_rel_turn_angle}\r\n")
    # my_uart3.write(f",{my_vision_manager.target_rel_speed_x},{my_vision_manager.target_rel_speed_y}\r\n")
    # my_uart2.write(f"{pose_data.now_pitch},{pose_data.now_roll},{pose_data.now_yaw},{pose_data.acc_x},{pose_data.acc_y},{pose_data.acc_z},{pose_data.gyro_x},{pose_data.gyro_y},{pose_data.gyro_z}\n")
    # my_uart3.write(f"{my_moving.current_state},{my_vision_manager.if_lost_object}\r\n")
    # my_uart2.write(f"{my_car.x_current},{my_car.y_current}\n")
    # my_uart3.write(f"{my_plan.target_v},{my_plan.target_yaw},{my_car.now_yaw * 180 / PI}\n")
    # my_uart3.write(f"{pose_data.now_pitch},{pose_data.now_roll},{pose_data.now_yaw},{pose_data.gyro_x},{pose_data.gyro_y},{pose_data.gyro_z},{my_car.now_yaw * 180 / PI}\n")
    # my_uart3.write(f"servo_pid.target_y: {servo_pid.target_y}, object_radius: {my_vision_manager.orbit_radius}\n")
    # my_uart3.write(f"state: {my_state.state}\n")
    # my_uart3.write(f"{my_vision_manager.current_servo_object}\r\n")
    # my_uart3.write(f"{pose_data.now_pitch},{pose_data.now_roll},{pose_data.now_yaw},{pose_data.gyro_z}\n")
    # my_uart3.write(f"{my_car.now_yaw * 180 / PI}\n")

# 定时器1初始化（中断回调函数在 ant_motor 中）
def pit1_start():
    global imu_data, pit1
    pit1.capture_list(imu, encoder_ul, encoder_ur, encoder_md)
    # 进行IMU零漂校准并将imu_data与定时器1的底层采集绑定
    pose_data.init_bias()
    pit1.callback(time_pit1_handler)
    pit1.start(motor_control_T)

# 定时器2初始化（中断回调函数在 ant_menu 中）
def pit2_start():
    global pit2
    pit2.callback(time_pit2_handler)
    pit2.capture_list(key)
    pit2.start(uart_and_menu_T)

# 定时器3初始化（中断回调函数在 ant_plan 中）
def pit3_start():
    global pit3
    pit3.callback(time_pit3_handler)
    pit3.start(plan_calculate_T)

###################################【主程序模块】###################################
# 检测电源电压是否正常
voltage_detect(11.2)

# 打开定时器
pit2_start()

while True:
    # I2C访问可能阻塞，必须放在普通主循环中，避免拖住姿态和电机定时器。
    if not if_menu:
        my_tof.service()

    gc.collect()
