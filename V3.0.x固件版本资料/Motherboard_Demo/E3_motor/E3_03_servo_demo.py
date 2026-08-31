
# 本示例程序演示如何使用 machine 库的 PWM 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的舵机接口

# 示例程序运行效果为每 50ms(0.05s) 改变一次 B26 引脚输出的占空比
# 对应拓展学习板供电并接接入舵机后 舵机将会以 10s 为周期来回摆动
# 每次改变舵机动作方向时 C4 LED 灯的亮灭状态改变一次
# 当 D9 引脚电平出现变化时退出测试程序

# !!!!! 请务必注意 !!!!! 禁止安装舵机摆臂连接前轮测试 !!!!! 
# !!!!! 请务必注意 !!!!! 禁止安装舵机摆臂连接前轮测试 !!!!! 
# !!!!! 请务必注意 !!!!! 禁止安装舵机摆臂连接前轮测试 !!!!! 

# 这是为了避免没有调整过舵机中值先装舵机 导致舵机角度与摆臂限幅冲突
# 错误的安装步骤存在舵机堵转的风险 舵机堵转会导致舵机损坏

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc, time

# 延迟上电 避免 CR 时序控制导致屏幕还未成功启动
time.sleep_ms(100)

# 核心板上 C4 是 LED
# 学习板上 D9  对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 定义一个角度与占空比换算的函数 传入参数为 PWM 的频率和目标角度
# 计算公式为 (duty_max / (1000ms / freq_Hz)) * (0.5 + angle / 90.0)
# duty_max / (1000ms / freq_Hz) 得到每毫秒对应的占空比数值
# (0.5 + angle / 90.0) 得到角度对应的毫秒数 舵机是 0-180 对应 0.5ms-2.5ms
# int(x) 接口用于将 x 转换为整数数值 PWM 接口需要整数输入 所以返回值需要转换为整数
def duty_angle (freq, angle):
    return int(65535.0 / (1000.0 / freq) * (0.5 + angle / 90.0))

# 使用 300Hz 的舵机控制频率
pwm_servo_hz = 300
# 初始角度 90 度 也就是舵机中值角度
angle = 90.0
# 舵机动作方向
dir = 1
# 获取舵机中值角度对应占空比
duty = duty_angle(pwm_servo_hz, angle)

# ------------------------------------------------------------------------------
#   构造接口 是标准 MicroPython 的 machine.PWM 模块
#   PWM_obj = PWM(pin, freq, [duty])
#       pin             引脚名称    |   必要参数 对应核心板上有 PWM 功能的引脚
#       freq            信号频率    |   必要参数
#       duty            占空比值    |   必要参数 关键字输入 范围 [1, 65535]
#       return          返回内容    |   正常情况下返回对应 PWM 通道的对象
# ------------------------------------------------------------------------------
pwm_servo = PWM("B26", pwm_servo_hz, duty_u16 = duty)
# 学习板上舵机接口为 B26

# PWM 接口 :
# ------------------------------------------------------------------------------
#   占空比值接口
#   PWM.duty_u16(duty)
#   duty = PWM.duty_u16()
#       duty            占空比值    |   可选参数 输入 duty 则更新占空比设置 范围 [1, 65535]
#       return          返回内容    |   当不输入 duty 参数时 返回当前占空比值 否则无返回内容
# ------------------------------------------------------------------------------
#   信号频率接口
#   PWM.freq(freq)
#   freq = PWM.freq()
#       freq            信号频率    |   可选参数 输入 freq 则更新频率设置
#       return          返回内容    |   当不输入 freq 参数时 返回当前信号频率 否则无返回内容
# ------------------------------------------------------------------------------

while True:
    # 延时 50 ms
    time.sleep_ms(50)
    # 往复计算舵机角度
    if dir:
        angle = angle + 0.1
        if angle >= 95.0:
            dir = 0
            led.toggle()
    else:
        angle = angle - 0.1
        if angle <= 85.0:
            dir = 1
            led.toggle()
    # 获取舵机角度对应占空比
    duty = duty_angle(pwm_servo_hz, angle)
    
    pwm_servo.duty_u16(duty)
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
