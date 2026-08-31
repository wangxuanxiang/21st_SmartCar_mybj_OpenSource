import micropython
from micropython import const
import math
import time
import gc

PI = const(3.1415926)
OneThird = const(0.3333333)
SQRT3 = const(1.7320508)
InField = const(-1)
OnLine = const(0)
OutLine = const(1)

# 光电管控制类
class PhotoControl:
    def __init__(self, flash_sys, beep, photo) -> None:
        self.flash_sys = flash_sys
        self.my_beep = beep
        self.my_photo = photo
        self.photo_state = self.my_photo.value()
        self.current_state = InField
        # 当光电管位于黄线正上方的次数
        self.on_line_times = 0

        gc.collect()

    def update_photo_state(self):
        current_state = self.my_photo.value()
        if current_state == 1 and self.current_state == InField:
            self.on_line_times += 1
            if self.on_line_times >= 3:  # 连续3次检测到在线，才认为真正进入了线上
                self.on_line_times = 0
                self.current_state = OnLine
        else:
            self.on_line_times = 0
        
        if current_state == 0 and self.current_state == OnLine:
            self.current_state = OutLine

    def reset_photo(self):
        self.on_line_times = 0
        self.current_state = InField
        

# 无刷风扇控制类
class FanControl:
    def __init__(self, flash_sys, fan , state):
        self.flash_sys = flash_sys
        self.my_fan = fan
        self.my_state = state

        self.fan_signal_limit = 1500  # type: int  # 无刷风扇信号限幅
        self.if_fan = self.flash_sys.find_value("if_fan")  # type: bool  # 是否开启风扇控制
        self.fixed_high_level_us = self.flash_sys.find_value("fixed_high_level_us")  # type: int  # 高电平持续时间，单位微秒

        gc.collect()

    # 设置无刷风扇的高电平时间
    def set_fan_signal(self):
        # 限幅在 1000-self.fan_signal_limit 之间
        if self.if_fan:
            high_level_us = max(1000, min(self.fixed_high_level_us, self.fan_signal_limit)) 
            # 更新高电平时间值
            self.my_fan.highlevel_us(high_level_us)
        else:
            self.fan_off()

    # 测试用的风扇高电平时间设置函数，直接传入一个值进行测试
    def test_fan(self, high_level_us):
        high_level_us = max(1000, min(high_level_us, self.fan_signal_limit)) 
        self.my_fan.highlevel_us(high_level_us)

    # 关闭风扇（设置为最低信号）
    def fan_off(self):
        self.my_fan.highlevel_us(1000)

class PID_data:
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys

        self.ul_high_kp = self.flash_sys.find_value("ul_high_kp")  # type: float
        self.ul_high_ki = self.flash_sys.find_value("ul_high_ki")  # type: float
        self.ul_high_kd = self.flash_sys.find_value("ul_high_kd")  # type: float
        self.ur_high_kp = self.flash_sys.find_value("ur_high_kp")  # type: float
        self.ur_high_ki = self.flash_sys.find_value("ur_high_ki")  # type: float
        self.ur_high_kd = self.flash_sys.find_value("ur_high_kd")  # type: float
        self.md_high_kp = self.flash_sys.find_value("md_high_kp")  # type: float
        self.md_high_ki = self.flash_sys.find_value("md_high_ki")  # type: float
        self.md_high_kd = self.flash_sys.find_value("md_high_kd")  # type: float

        self.ul_mid_kp = self.flash_sys.find_value("ul_mid_kp")  # type: float
        self.ul_mid_ki = self.flash_sys.find_value("ul_mid_ki")  # type: float
        self.ul_mid_kd = self.flash_sys.find_value("ul_mid_kd")  # type: float
        self.ur_mid_kp = self.flash_sys.find_value("ur_mid_kp")  # type: float
        self.ur_mid_ki = self.flash_sys.find_value("ur_mid_ki")  # type: float
        self.ur_mid_kd = self.flash_sys.find_value("ur_mid_kd")  # type: float
        self.md_mid_kp = self.flash_sys.find_value("md_mid_kp")  # type: float
        self.md_mid_ki = self.flash_sys.find_value("md_mid_ki")  # type: float
        self.md_mid_kd = self.flash_sys.find_value("md_mid_kd")  # type: float
        
        self.ul_low_kp = self.flash_sys.find_value("ul_low_kp")  # type: float
        self.ul_low_ki = self.flash_sys.find_value("ul_low_ki")  # type: float
        self.ul_low_kd = self.flash_sys.find_value("ul_low_kd")  # type: float
        self.ur_low_kp = self.flash_sys.find_value("ur_low_kp")  # type: float
        self.ur_low_ki = self.flash_sys.find_value("ur_low_ki")  # type: float
        self.ur_low_kd = self.flash_sys.find_value("ur_low_kd")  # type: float
        self.md_low_kp = self.flash_sys.find_value("md_low_kp")  # type: float
        self.md_low_ki = self.flash_sys.find_value("md_low_ki")  # type: float
        self.md_low_kd = self.flash_sys.find_value("md_low_kd")  # type: float

        gc.collect()  # 主动触发垃圾回收，释放内存

# 滑动平均滤波器
class SlipAveragingFilter:
    # 构造对象时传入滤波窗口大小
    def __init__(self, filter_size: int):
        self.filter_size = filter_size
        self.index = 0
        self.last_value = 0.0
        self.buffer = [0.0] * filter_size

        gc.collect()

    def buffer_init(self, initial_value):
        self.buffer = [initial_value] * self.filter_size

    # 滤波时传入一个新的数据，返回滤波后的结果(float)
    def filtering(self, data: float) -> float:
        self.buffer[self.index] = data
        self.index = (self.index + 1) % self.filter_size
        return sum(self.buffer) / self.filter_size
    
# 一维卡尔曼滤波器
class KalmanFilter:
    def __init__(self, P=1.0, Q=0.01, R=0.1, initial_output=0.0):
        self.P = P
        self.Q = Q
        self.R = R
        self.Output = initial_output

        gc.collect()

    def update(self, input_value):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.Output += K * (input_value - self.Output)
        self.P = (1 - K) * self.P
        return self.Output    
    
class PoseData:
    def __init__(self, flash_sys, my_uart2, imu, encoder_ul, encoder_ur, encoder_md, diff_filter_gyroz, acc_x_filter, acc_y_filter, acc_z_filter):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入串口对象
        self.my_uart2 = my_uart2
        # 注入传感器对象
        self.imu = imu
        self.encoder_ul = encoder_ul
        self.encoder_ur = encoder_ur
        self.encoder_md = encoder_md
        # 加速度计滤波器
        self.acc_x_filter = acc_x_filter
        self.acc_y_filter = acc_y_filter
        self.acc_z_filter = acc_z_filter
        # 注入滤波器对象
        self.diff_filter_gyroz = diff_filter_gyroz
        # IMU数据列表
        self.imu_data = []   # type: list

        # 传感器数据
        self.encoder_data_ul = 0    # type: int
        self.encoder_data_ur = 0    # type: int
        self.encoder_data_md = 0    # type: int
        
        # 陀螺仪补偿系数
        self.gyro_z_supply = self.flash_sys.find_value("gyro_z_supply")
        # 加速度
        self.acc_x = 0              # type: float
        self.acc_y = 0              # type: float
        self.acc_z = 0              # type: float
        # 角速度
        self.gyro_x = 0             # type: float
        self.gyro_y = 0             # type: float
        self.gyro_z = 0             # type: float
        self.gyro_z_gkd = 0         # type: float # 供角速度环控制用的原始角速度值
        # 角速度零漂误差
        self.gyro_x_bias = 0.0       # type: float
        self.gyro_y_bias = 0.0       # type: float
        self.gyro_z_bias = 0.0        # type: float

        # 四元数初始化
        self.q = [1.0, 0.0, 0.0, 0.0]
        # 误差积分项
        self.e_int = [0.0, 0.0, 0.0]
        
        # 上次更新时间戳
        self.last_update_time = time.ticks_us()

        # 算法参数 (根据你的 4ms 采样周期设置)
        self.dt = 0.004 
        self.kp = 1.0  # 加速度计权重
        self.ki = 0.001 # 零偏补偿权重

        # 最终角度输出
        self.now_pitch = 0.0  # 俯仰角
        self.now_roll = 0.0   # 横滚角
        self.now_yaw = 0.0    # 偏航角

        gc.collect()  # 主动触发垃圾回收，释放内存

        # 更新四元数
    def ahrs_update(self, ax, ay, az, gx, gy, gz):
        """
        核心四元数更新算法
        输入单位：ax-az (g), gx-gz (rad/s)
        """
        ax = self.acc_x_filter.filtering(ax)
        ay = self.acc_y_filter.filtering(ay)
        az = self.acc_z_filter.filtering(az)

        self.acc_x = ax
        self.acc_y = ay
        self.acc_z = az

        q0, q1, q2, q3 = self.q
        
        # 1. 当前加速度计的原始数据模长
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0: return # 防止除以0

        G_REFERENCE = 4170.0  # TODO: 请你在串口打印一下静止时 norm 的值，并把它填在这里！
    
        # 计算测量模长与标准重力 1g 的绝对偏差 (单位重新化为 g)
        acc_error = abs(norm - G_REFERENCE) / G_REFERENCE
        
        # self.my_uart2.write(f"{acc_error}\n")  # 调试用：输出加速度模长偏差

        # --- norm 偏差权重（加速度模长与重力的偏差）---
        # 偏差在 5% 以内完全信任，偏差大于 20% 完全不信任
        LOWER_THRESHOLD = 0.05
        UPPER_THRESHOLD = 0.20

        if acc_error < LOWER_THRESHOLD:
            norm_weight = 1.0
        elif acc_error > UPPER_THRESHOLD:
            norm_weight = 0.0
        else:
            # 线性插值，平滑过渡
            norm_weight = 1.0 - ((acc_error - LOWER_THRESHOLD) / (UPPER_THRESHOLD - LOWER_THRESHOLD))

        # --- 水平加速度幅值权重（防止急转弯时加速度计"骗"了姿态）---
        # 低于 200 不干预，高于 700 才完全拉黑，中间线性过渡
        ACC_LOWER = 200
        ACC_UPPER = 700

        abs_ax = abs(ax)
        abs_ay = abs(ay)

        if abs_ax < ACC_LOWER:
            ax_weight = 1.0
        elif abs_ax > ACC_UPPER:
            ax_weight = 0.0
        else:
            ax_weight = 1.0 - (abs_ax - ACC_LOWER) / (ACC_UPPER - ACC_LOWER)

        if abs_ay < ACC_LOWER:
            ay_weight = 1.0
        elif abs_ay > ACC_UPPER:
            ay_weight = 0.0
        else:
            ay_weight = 1.0 - (abs_ay - ACC_LOWER) / (ACC_UPPER - ACC_LOWER)

        mag_weight = ax_weight if ax_weight < ay_weight else ay_weight

        # 取两种机制中最严格的权重（双重保险）
        dynamic_weight = min(norm_weight, mag_weight)
            
        # 计算当前周期实际使用的 kp
        current_kp = self.kp * dynamic_weight

        # self.my_uart2.write(f"{dynamic_weight},{current_kp}\n")  # 调试用：输出动态权重和当前 kp

        # self.my_uart3.write(f"{norm},{current_kp}\n")  # 调试用：输出原始加速度模长

        # 继续执行归一化，将向量化为长度为 1 的单位向量给后续解算用
        ax /= norm; ay /= norm; az /= norm
        
        # 2. 提取四元数矩阵中的理论重力方向 (机体坐标系下)
        vx = 2 * (q1*q3 - q0*q2)
        vy = 2 * (q0*q1 + q2*q3)
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3
        
        # 3. 叉乘计算误差 (测量值与理论值的偏差)
        ex = (ay*vz - az*vy)
        ey = (az*vx - ax*vz)
        ez = (ax*vy - ay*vx)
        
        # --- 改进1：增加积分限幅 (Anti-Windup) ---
        # 6轴系统不修正 Yaw 的积分，始终清零
        self.e_int[2] = 0.0

        # 只有加速度计可信时才更新积分项，否则冻结，防止累积"垃圾"
        if dynamic_weight > 0.1:
            I_LIMIT = 0.1  # 限制积分项最大影响
            self.e_int[0] = max(-I_LIMIT, min(self.e_int[0] + ex * self.ki, I_LIMIT))
            self.e_int[1] = max(-I_LIMIT, min(self.e_int[1] + ey * self.ki, I_LIMIT))
        
        # 强制将 ez 置为 0，防止加速度计在 Z 轴上的假误差污染陀螺仪的 gz
        ez = 0.0 

        # --- 改进2：补偿角速度 ---
        gx += current_kp * ex + self.e_int[0]
        gy += current_kp * ey + self.e_int[1]
        gz += current_kp * ez + self.e_int[2]

        # 6. 一阶龙格库塔法更新四元数
        half_dt = 0.5 * self.dt
        q0_new = q0 + (-q1*gx - q2*gy - q3*gz) * half_dt
        q1_new = q1 + (q0*gx + q2*gz - q3*gy) * half_dt
        q2_new = q2 + (q0*gy - q1*gz + q3*gx) * half_dt
        q3_new = q3 + (q0*gz + q1*gy - q2*gx) * half_dt
        
        # 7. 再次归一化四元数
        norm = math.sqrt(q0_new*q0_new + q1_new*q1_new + q2_new*q2_new + q3_new*q3_new)
        self.q[0] = q0_new/norm
        self.q[1] = q1_new/norm
        self.q[2] = q2_new/norm
        self.q[3] = q3_new/norm
    
    # 将四元数转化为欧拉角
    def update_euler_angles(self):
        """将四元数转换为欧拉角（度）"""
        q0, q1, q2, q3 = self.q
        
        val = -2.0 * (q1 * q3 - q0 * q2)

        val = max(-1.0, min(1.0, val))
        self.now_pitch = math.asin(val) * (180.0 / PI)

        if abs(val) > 0.999: # 极高仰角时 Roll 和 Yaw 共线
            self.now_roll = 0.0
            self.now_yaw = math.atan2(2.0 * (q1 * q2 - q0 * q3), 1.0 - 2.0 * (q1 * q1 + q3 * q3)) * (180.0 / PI)
        else:
            self.now_roll = math.atan2(2.0 * (q2 * q3 + q0 * q1), 
                                    1.0 - 2.0 * (q1 * q1 + q2 * q2)) * (180.0 / PI)
            
            self.now_yaw = math.atan2(2.0 * (q1 * q2 + q0 * q3), 
                                    1.0 - 2.0 * (q2 * q2 + q3 * q3)) * (180.0 / PI)

    # 重置四元数
    def reset_yaw(self, ref_yaw_deg):
        """
        通过外部参考信息强制重置当前的偏航角 (Yaw)。
        保留当前的横滚角 (Roll) 和俯仰角 (Pitch)，重新合成四元数。
        
        :param ref_yaw_deg: 外部传感器获取的绝对偏航角，单位：度 (°)
        """
        # 1. 将角度转换为半角弧度

        # 重置俯仰角和偏航角
        self.now_roll = 0.0
        self.now_pitch = 0.0

        half_roll = self.now_roll * 0.5 * (PI / 180.0)
        half_pitch = self.now_pitch * 0.5 * (PI / 180.0)
        half_yaw = -ref_yaw_deg * 0.5 * (PI / 180.0)

        # 2. 预计算三角函数以提高运算效率
        sr = math.sin(half_roll)
        cr = math.cos(half_roll)
        sp = math.sin(half_pitch)
        cp = math.cos(half_pitch)
        sy = math.sin(half_yaw)
        cy = math.cos(half_yaw)

        # 3. 欧拉角转四元数 (基于你原始解算的 Z-Y-X 旋转顺序)
        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy

        # 为了确保精度，再次对生成的四元数进行归一化
        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        if norm == 0:
            return # 防止除零异常

        # 4. 强制覆盖当前四元数状态
        self.q[0] = q0/norm
        self.q[1] = q1/norm
        self.q[2] = q2/norm
        self.q[3] = q3/norm

        # 5. 清空 PI 算法的积分补偿项
        # 这一步极其重要：如果不清空，历史误差的积分积累会在接下来的几个周期内把姿态又“拉回”一点点，导致修正不干脆
        self.e_int[0] = 0.0
        self.e_int[1] = 0.0
        self.e_int[2] = 0.0

        # 6. 同步更新底层的欧拉角输出，确保下一个控制周期读取的数据是最新值
        self.update_euler_angles()

    # 初始零偏计算函数，总计需延时3s，初始化陀螺仪的同时进行启动延时，确保平稳启动
    def init_bias(self):
        gyro_x_sum = 0
        gyro_y_sum = 0
        gyro_z_sum = 0
        sample_count = 500
        # 将imu_data与imu对象链接起来
        self.imu_data = self.imu.get()
        for i in range(sample_count):
            self.imu_data = self.imu.read()
            gyro_x_sum += self.imu_data[3]
            gyro_y_sum += self.imu_data[4]
            gyro_z_sum += self.imu_data[5]
            time.sleep_ms(4)  # 延时2ms，确保采样间隔均匀

        self.gyro_x_bias = gyro_x_sum / sample_count    
        self.gyro_y_bias = gyro_y_sum / sample_count
        self.gyro_z_bias = gyro_z_sum / sample_count


    # 传感器数据更新函数
    def update_data(self):
        """
        # 1. 计算真实的动态 dt
        current_time = time.ticks_us()
        # 计算时间差并转换为秒 (MicroPython 下推荐用 ticks_diff 防溢出)
        self.dt = time.ticks_diff(current_time, self.last_update_time) / 1000000.0
        self.last_update_time = current_time

        # self.my_uart3.write(f"dt: {self.dt:.6f} s\n")  # 调试用：输出实际 dt
        # 防止 dt 出现离谱的值（比如程序刚启动卡顿）
        if self.dt > 0.1: 
            self.dt = 0.004
        """
            
        self.encoder_data_ul = self.encoder_ul.get()
        self.encoder_data_ur = self.encoder_ur.get()
        self.encoder_data_md = self.encoder_md.get()
        
        self.gyro_x = (self.imu_data[3] - self.gyro_x_bias) / 16.4 * (PI / 180.0) * self.gyro_z_supply
        self.gyro_y = (self.imu_data[4] - self.gyro_y_bias) / 16.4 * (PI / 180.0) * self.gyro_z_supply
        self.gyro_z = (self.imu_data[5] - self.gyro_z_bias) / 16.4 * (PI / 180.0) * self.gyro_z_supply
        # self.gkd用于角速度环控制
        self.gyro_z_gkd = -self.gyro_z * (180.0 / PI)

        DEADBAND = 0.004 # 弧度每秒
        if abs(self.gyro_x) < DEADBAND: self.gyro_x = 0.0
        if abs(self.gyro_y) < DEADBAND: self.gyro_y = 0.0
        if abs(self.gyro_z) < DEADBAND: self.gyro_z = 0.0

        self.ahrs_update(self.imu_data[0], self.imu_data[1], self.imu_data[2],
                        self.gyro_x, self.gyro_y, self.gyro_z)

        # 4. 更新欧拉角输出
        self.update_euler_angles()

        
# 定义一个抽象类用于顶层设计
# 该类能够存储pid参数并计算得到当前应该输出的pwm值
class ControlPID:
    def compute_pid(self, target: int, actual: int) -> None:
        pass

# 速度环位置式PID
class SpeedPositionPID(ControlPID):
    def __init__(self, flash_sys, diff_filter: SlipAveragingFilter):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        self.kp = 0.0        # type: float
        self.ki = 0.0       # type: float
        self.kd = 0.0       # type: float
        # 速度前馈系数
        self.kv = self.flash_sys.find_value("kv")  # type: float
        self.target = 0     # type: float
        self.actual = 0     # type: int
        self.nowError = 0   # type: float
        self.preError = 0   # type: float
        self.integral = 0   # type: float
        self.derivative = 0 # type: float
        self.pwm_output = 0 # type: float
        self.__integral_limitmax = self.flash_sys.find_value("integral_limitmax")      # type: float
        self.__pwmout_limitmax = self.flash_sys.find_value("pwmout_limitmax")          # type: float
        # 注入微分项滤波器对象
        self.diff_filter = diff_filter
        self.__A = self.flash_sys.find_value("A")      # type: float # 变速积分误差阈值上限
        self.__B = self.flash_sys.find_value("B")      # type: float # 变速积分误差阈值下限

    def set_pid_params(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute_pid(self, target: float, actual: int):
        self.target = target
        self.actual = actual
        self.preError = self.nowError
        self.nowError = self.target - self.actual

        abs_nowerror = abs(self.nowError)
        coefficient = 1.0   # type: float
        if self.__A == self.__B:
            # 避免除以0
            if (abs_nowerror > self.__A):
                coefficient = 0.0
            else:
                coefficient = 1.0
        else:
            if abs_nowerror > self.__A:
                coefficient = 0.0
            elif abs_nowerror > self.__B:
                coefficient = (self.__A - abs_nowerror) / (self.__A - self.__B)
            else:
                coefficient = 1.0
        
        # 根据误差大小调整积分项
        self.integral += coefficient * self.nowError

        # 积分项限幅
        self.integral = max(-self.__integral_limitmax, min(self.integral, self.__integral_limitmax))

        # 对微分项进行滑动平均滤波
        self.derivative = self.diff_filter.filtering(self.nowError - self.preError)

        # 计算pwm_output
        self.pwm_output = self.kp * self.nowError+ self.ki * self.integral + self.kd * self.derivative + self.kv * self.target
        
        # 当目标速度为0且此时误差极小时，强制增加一个制动pwm输出来驱动
        if self.target == 0:
            if self.nowError < 5 and self.nowError > 0:
                self.pwm_output += self.pwm_output + 500
            elif self.nowError > -5 and self.nowError < 0:
                self.pwm_output += self.pwm_output - 500
    
        # pwm_output限幅
        self.pwm_output = max(-self.__pwmout_limitmax, min(self.pwm_output, self.__pwmout_limitmax))

    # 清零积分项
    def reset_integral(self):
        self.integral = 0

# 角度环PID
class AnglePositionPID(ControlPID):
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        self.kp = self.flash_sys.find_value("angle_normal_kp")        # type: float
        self.kd = self.flash_sys.find_value("angle_normal_kd")        # type: float
        self.angle_normal_kp = self.flash_sys.find_value("angle_normal_kp")        # type: float
        self.target = 0     # type: float
        self.actual = 0     # type: float
        self.nowError = 0   # type: float
        self.preError = 0   # type: float
        self.integral = 0   # type: float
        self.derivative = 0 # type: float
        self.pwm_output = 0 # type: float
        self.high_pwmout_limitmax = self.flash_sys.find_value("high_angle_pwmout_limitmax")    # type: float
        self.low_pwmout_limitmax = self.flash_sys.find_value("low_angle_pwmout_limitmax")    # type: float
        self.pwmout_limitmax = self.high_pwmout_limitmax

        # 选择大角度闭环还是小角度
        self.if_high_angle = False
        # 角度展开：追踪连续的实际角度（跨越±180边界时累积偏移）
        self.actual_unwrap_offset = 0.0   # type: float
        self.prev_actual = 0.0            # type: float
        self.effective_target = 0.0       # type: float  # 大角度模式下的展开目标值
        self.prev_target = 0.0            # type: float  # 上一周期的target，用于检测target是否变化

    def compute_pid(self, target: float, actual: float):
        self.target = target
        self.actual = actual
        self.preError = self.nowError

        # 角度展开：检测IMU跨越±180边界，累积偏移量
        actual_diff = self.actual - self.prev_actual
        if actual_diff > 180.0:
            self.actual_unwrap_offset -= 360.0
        elif actual_diff < -180.0:
            self.actual_unwrap_offset += 360.0
        self.prev_actual = self.actual
        actual_unwrapped = self.actual + self.actual_unwrap_offset

        if not self.if_high_angle:
            # 小角度模式：将误差归一化到 [-180, 180]
            self.nowError = self.target - self.actual
            if self.nowError > 180:
                self.nowError -= 360
            elif self.nowError < -180:
                self.nowError += 360
        else:
            # 大角度模式：用展开后的连续角度计算误差，避免±180边界跳变
            # 计算短路径误差
            short_error = self.target - self.actual
            if short_error > 180.0:
                short_error -= 360.0
            elif short_error < -180.0:
                short_error += 360.0

            if abs(short_error) <= 10.0:
                # 死区内：已接近目标，直接用短路径收敛
                self.nowError = short_error
                self.effective_target = 0.0  # 下次换target时重新计算
            else:
                # 远路目标：选离 actual_unwrapped 较远的一侧（>180°的路径）
                # 检测target是否变化：变了才重新计算effective_target
                if abs(self.target - self.prev_target) > 0.5:
                    # 找到 target 在展开坐标中离 actual_unwrapped 最近的等价位置（= 短路径）
                    wraps = (actual_unwrapped - self.target) / 360.0
                    # MicroPython 兼容：手动四舍五入到最近整数
                    if wraps >= 0:
                        wraps_int = int(wraps + 0.5)
                    else:
                        wraps_int = int(wraps - 0.5)
                    closest = self.target + wraps_int * 360.0
                    # 从 closest 向远路方向再偏移一圈
                    if short_error > 0.0:
                        # 短路径是 CW(+)，远路走 CCW(-)
                        self.effective_target = closest - 360.0
                    else:
                        # 短路径是 CCW(-)，远路走 CW(+)
                        self.effective_target = closest + 360.0
                # target不变时effective_target保持锁定，靠展开角度保证连续性
                # 用展开后的实际角度计算连续误差
                self.nowError = self.effective_target - actual_unwrapped
            self.prev_target = self.target
                    
        self.integral += self.nowError
        self.derivative = self.nowError - self.preError
        # 归一化derivative，消除大角度模式下±180°边界跳变导致的尖峰
        if self.derivative > 180:
            self.derivative -= 360
        elif self.derivative < -180:
            self.derivative += 360

        # 计算pwm_output
        self.pwm_output = self.kp * self.nowError + self.kd * self.derivative

        # pwm_output限幅
        self.pwm_output = max(-self.pwmout_limitmax, min(self.pwm_output, self.pwmout_limitmax))

    # 是否选择大角度模式
    def choose_high_angle_mode(self, if_high_angle: bool):
        self.if_high_angle = if_high_angle
        if if_high_angle:
            # 进入大角度模式时重置，下次 compute_pid 会重新计算 effective_target
            self.effective_target = 0.0
            self.prev_target = 0.0

    # 判断小车是否完成转角
    def if_finish_turn(self):
        return abs(self.nowError) < 2.0
    
# 视觉伺服PD
class ServoPID(ControlPID):
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        self.servo_kp_normal_x = self.flash_sys.find_value("servo_kp_normal_x")        # type: float
        self.servo_kd_normal_x = self.flash_sys.find_value("servo_kd_normal_x")        # type: float
        self.servo_kp_normal_y = self.flash_sys.find_value("servo_kp_normal_y")        # type: float
        self.servo_kd_normal_y = self.flash_sys.find_value("servo_kd_normal_y")    
        self.servo_kp_x = 0.0
        self.servo_kp_y = 0.0
        self.servo_kd_x = 0.0
        self.servo_kd_y = 0.0
        self.target_x = 0.0
        self.actual_x = 0.0
        self.target_y = 0.0   # type: float
        self.actual_y = 0.0     # type: float

        self.target_y_T = self.flash_sys.find_value("servo_target_y_T")     # type: float
        self.target_y_S = self.flash_sys.find_value("servo_target_y_S")     # type: float
        self.target_y_B = self.flash_sys.find_value("servo_target_y_B")     # type: float    # type: float
        self.current_obj = '' # type: str 
        self.nowError_x = 0   # type: float
        self.preError_x = 0   # type: float
        self.nowError_y = 0   # type: float
        self.preError_y = 0   # type: float
        self.derivative_x = 0 # type: float
        self.derivative_y = 0 # type: float
        self.pwm_output_x = 0 # type: int
        self.pwm_output_y = 0 # type: int
        self.__pwmout_limitmax = self.flash_sys.find_value("servo_pwmout_limitmax")    # type: int
    
        gc.collect()  # 主动触发垃圾回收，释放内存

    # 模型下的pid计算
    def model_compute_pid(self, actual_x: float, actual_y: float):
        self.actual_x = actual_x
        self.actual_y = actual_y    
        self.preError_x = self.nowError_x
        self.preError_y = self.nowError_y
        self.nowError_x = self.actual_x - self.target_x 
        self.nowError_y = self.actual_y - self.target_y
        # 计算微分项
        self.derivative_x = self.nowError_x - self.preError_x
        self.derivative_y = self.nowError_y - self.preError_y
        # 计算pwm_output
        self.pwm_output_x = int(self.servo_kp_x * self.nowError_x + self.servo_kd_x * self.derivative_x)
        self.pwm_output_y = int(self.servo_kp_y * self.nowError_y + self.servo_kd_y * self.derivative_y)

        # pwm_output限幅
        self.pwm_output_x = max(-self.__pwmout_limitmax, min(self.pwm_output_x, self.__pwmout_limitmax))
        self.pwm_output_y = max(-self.__pwmout_limitmax, min(self.pwm_output_y, self.__pwmout_limitmax))

# 距离控制PD
class DistPID(ControlPID):
    def __init__(self, flash_sys, L_or_R: str, filter: SlipAveragingFilter):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        self.dist_kp = self.flash_sys.find_value("dist_kp")        # type: float
        self.dist_kd = self.flash_sys.find_value("dist_kd")        # type: float
        self.target_T = self.flash_sys.find_value("target_T")
        self.target_S = self.flash_sys.find_value("target_S")
        self.target_B = self.flash_sys.find_value("target_B")
        self.L_or_R = L_or_R
        self.my_filter = filter
        self.target =  0.0     # type: float
        self.actual = 0.0
        self.deadzone = self.flash_sys.find_value("dist_deadzone")     # type: float
        self.nowError = 0   # type: float
        self.preError = 0   # type: float
        self.derivative = 0 # type: float
        self.pwm_output = 0 # type: int
        self.__pwmout_limitmax = self.flash_sys.find_value("dist_pwmout_limitmax")    # type: int
    
        gc.collect()  # 主动触发垃圾回收，释放内存

    # pid计算
    def compute_pid(self, actual: float):
        self.actual = actual
        self.preError = self.nowError
        self.nowError = self.actual - self.target

        # 死区判断：误差绝对值小于死区时，pwm输出置零
        if abs(self.nowError) < self.deadzone:
            self.pwm_output = 0
            return

        # 计算微分项
        self.derivative = self.nowError - self.preError
        # 计算pwm_output
        if self.L_or_R == "L":
            self.pwm_output = -int(self.dist_kp * self.nowError + self.dist_kd * self.derivative)
        elif self.L_or_R == "R":
            self.pwm_output = int(self.dist_kp * self.nowError + self.dist_kd * self.derivative)

        # 对pwm_output进行滑动平均滤波
        self.pwm_output = int(self.my_filter.filtering(float(self.pwm_output)))

        # pwm_output限幅
        self.pwm_output = max(-self.__pwmout_limitmax, min(self.pwm_output, self.__pwmout_limitmax))

    # 输出清零
    def reset_pwmout(self):
        self.pwm_output = 0

    # 初始化滤波器
    def init_filter(self, data):
        self.my_filter.buffer_init(data)

    # 根据物体种类选择小车间距
    def choose_dist(self, obj):
        self.current_obj = obj
        if obj == 'T':
            self.target = self.target_T
        elif obj in ['S', 'E']:
            self.target = self.target_S
        elif obj in ['B', 'W']:
            self.target = self.target_B


# 小车姿态控制
class CarPose:
    def __init__(self, flash_sys, state_machine, pose_data: PoseData, car_yaw_filter: SlipAveragingFilter, angle_pid: AnglePositionPID,
                motor_ul_pid: SpeedPositionPID, motor_ur_pid: SpeedPositionPID, motor_md_pid: SpeedPositionPID, motor_ul, motor_ur, motor_md):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入速度与路径规划对象
        self.my_state = state_machine
        # 注入姿态数据对象
        self.pose_data = pose_data
        # 注入小车自转角角滑动平均滤波器对象
        self.car_yaw_filter = car_yaw_filter
        # 注入角度pid对象
        self.angle_pid = angle_pid
        # 注入电机pid对象
        self.motor_ul_pid = motor_ul_pid
        self.motor_ur_pid = motor_ur_pid
        self.motor_md_pid = motor_md_pid
        # 注入电机对象
        self.motor_ul = motor_ul
        self.motor_ur = motor_ur
        self.motor_md = motor_md

        # 上一次速度
        self.last_car_speed_x = 0.0  # type: float
        self.last_car_speed_y = 0.0  # type: float
        # 小车坐标系下的当前速度
        self.car_speed_x = 0.0  # type: float
        self.car_speed_y = 0.0  # type: float
        # 目标转角
        self.turn_angle_target = 0.0  # type: float
        # 速度系数
        self.speed_conversion_gamma = self.flash_sys.find_value("speed_conversion_gamma")   # 将速度单位转化为cm每秒
        self.gkd = self.flash_sys.find_value("gkd")  # type: float  # 角速度补偿系数
        self.speed_fuse_ratio = self.flash_sys.find_value("speed_fuse_ratio")  # type: float  # 速度融合系数
        # 依据角度的位置修正系数（常量）
        self.alpha_x = 1.0  # type: float
        self.alpha_y = 1.0  # type: float
        # 位置
        self.x_current = 0.0   # type: float
        self.y_current = 0.0   # type: float
        self.now_yaw = 0.0  # type: float
        self.last_gyro_z = 0.0  # type: float
        self.last_time = 0      # type: int

        # tof距离控制变量
        self.speed_weight = 0.0  # type: float
        self.fixed_direction = 0.0  # type: float
        self.if_control_dist = False  # type: bool

        # 测试一个电机的里程
        # self.encouder_ul = 0.0    
        # self.encouder_ur = 0.0
        # self.encouder_md = 0.0

        gc.collect()  # 主动触发垃圾回收，释放内存
    
    # 清空上一次速度
    def clear_last_car_speed(self):
        self.last_car_speed_x = 0.0
        self.last_car_speed_y = 0.0

    # 小车姿态更新
    def update_pose(self):
        ###################【速度计算】###################
        # 保存上一次速度
        self.last_car_speed_x = self.car_speed_x
        self.last_car_speed_y = self.car_speed_y
        # 测试一个电机的里程
        # self.encouder_ul += self.speed_conversion_gamma * self.pose_data.encoder_data_ul / 1000
        # self.encouder_ur += self.speed_conversion_gamma * self.pose_data.encoder_data_ur / 1000
        # self.encouder_md += self.speed_conversion_gamma * self.pose_data.encoder_data_md / 1000

        # 计算小车当前x,y速度（互补融合）
        # car_speed_x, car_speed_y 单位：厘米每5ms
        self.car_speed_x = self.speed_fuse_ratio * self.last_car_speed_x + (1 - self.speed_fuse_ratio) * (OneThird * (self.pose_data.encoder_data_ur + self.pose_data.encoder_data_ul - self.pose_data.encoder_data_md * 2)  * self.speed_conversion_gamma / 1000)
        self.car_speed_y = self.speed_fuse_ratio * self.last_car_speed_y + (1 - self.speed_fuse_ratio) * (OneThird * SQRT3 * (self.pose_data.encoder_data_ul - self.pose_data.encoder_data_ur)) * self.speed_conversion_gamma / 1000

        # 计算小车在世界坐标系下的偏航角
        self.now_yaw = -self.pose_data.now_yaw * PI / 180.0
        # 限定now_yaw在-2pi到2pi之间
        if self.now_yaw > PI:  self.now_yaw -= 2 * PI
        elif self.now_yaw < -PI:  self.now_yaw += 2 * PI
        # 转换到世界坐标系下的速度
        real_speed_x = self.car_speed_x * math.cos(self.now_yaw) + self.car_speed_y * math.sin(self.now_yaw)
        real_speed_y = -self.car_speed_x * math.sin(self.now_yaw) + self.car_speed_y * math.cos(self.now_yaw)

        ###################【位置计算】###################
        # 依据当前航向角调整位置修正系数（解决小车在不同方向上的编码器积分结果不一致问题）
        # 计算小车当前位置，根据运动方向选择补偿系数
        self.x_current += real_speed_x * self.alpha_x
        self.y_current += real_speed_y * self.alpha_y

    # 全向移动控制函数
    # 参数说明：move_speed_target单位：编码器脉冲， move_angle_target单位：度， turn_angle_target单位：度
    def move_ctrl(self, move_speed_target: float, move_angle_target: float, turn_angle_target: float):
        if move_speed_target == 0:  # 整车刹车指令，显式清零
            self.motor_ul_pid.reset_integral()
            self.motor_ur_pid.reset_integral()
            self.motor_md_pid.reset_integral()

        # 将目标转角和目标航向角限定在-180到180度之间
        turn_angle_target = ((turn_angle_target + 180.0) % 360.0) - 180.0

        move_angle_target = ((move_angle_target + 180.0) % 360.0) - 180.0

        # 设置目标转角
        self.turn_angle_target = turn_angle_target

        # 距离控制模式：根据speed_weight合成垂直于当前速度方向的分量
        if self.if_control_dist and self.speed_weight != 0.0:
            self.fixed_direction = ((self.fixed_direction + 180.0) % 360.0) - 180.0

            # 当前move_angle_target转弧度用于向量分解
            rad = self.fixed_direction * PI / 180.0
            # 将move_angle_target转换为弧度
            move_angle_target = move_angle_target * PI / 180
            # 原始速度在世界坐标系下的分量
            vx_orig = move_speed_target * math.sin(move_angle_target)
            vy_orig = move_speed_target * math.cos(move_angle_target)
            abs_w = abs(self.speed_weight)
            if self.speed_weight < 0:
                # 逆时针旋转90度
                vx_perp = -abs_w * math.cos(rad)
                vy_perp = abs_w * math.sin(rad)
            else:
                # 顺时针旋转90度：
                vx_perp = abs_w * math.cos(rad)
                vy_perp = -abs_w * math.sin(rad)
            # 合成新速度向量
            vx_new = vx_orig + vx_perp
            vy_new = vy_orig + vy_perp
            # 计算新的目标速度和目标角度
            move_speed_target = math.sqrt(vx_new * vx_new + vy_new * vy_new)
            move_angle_target = -math.atan2(-vx_new, vy_new) * 180.0 / PI
            move_angle_target = ((move_angle_target + 180.0) % 360.0) - 180.0

        # 将move_angle_target转换为弧度
        move_angle_target = move_angle_target * PI / 180
        
        # 转换到小车坐标系下的目标速度
        car_speed_x_target = move_speed_target * math.sin(move_angle_target - self.now_yaw)
        car_speed_y_target = move_speed_target * math.cos(move_angle_target - self.now_yaw)
        car_speed_w_target = self.angle_pid.pwm_output

        # 计算各个电机的目标速度
        motor_ul_speed_target = (car_speed_w_target * OneThird + (car_speed_x_target + car_speed_y_target * SQRT3) * 0.5 + self.pose_data.gyro_z_gkd * self.gkd)
        motor_ur_speed_target = (car_speed_w_target * OneThird + (car_speed_x_target - car_speed_y_target * SQRT3) * 0.5 + self.pose_data.gyro_z_gkd * self.gkd)
        motor_md_speed_target = (car_speed_w_target * OneThird - car_speed_x_target + self.pose_data.gyro_z_gkd * self.gkd)

        # 计算各个电机的pid得到pwm输出
        self.motor_ul_pid.compute_pid(motor_ul_speed_target, self.pose_data.encoder_data_ul)
        self.motor_ur_pid.compute_pid(motor_ur_speed_target, self.pose_data.encoder_data_ur)
        self.motor_md_pid.compute_pid(motor_md_speed_target, self.pose_data.encoder_data_md)

    # 设置电机pwm输出函数
    def set_motor_pwm(self):
        self.motor_ul.duty(int(self.motor_ul_pid.pwm_output))
        self.motor_ur.duty(int(self.motor_ur_pid.pwm_output))
        self.motor_md.duty(int(self.motor_md_pid.pwm_output))

    # pwm信号归零
    def pwm_stop(self):
        self.motor_ul.duty(0)
        self.motor_ur.duty(0)
        self.motor_md.duty(0)

    # 积分清零
    def reset_pid_integral(self):
        self.motor_ul_pid.reset_integral()
        self.motor_ur_pid.reset_integral()
        self.motor_md_pid.reset_integral()