
# 本示例程序演示如何使用 seekfree 库的 MOTOR_CONTROLLER 类接口
# 使用 RT1021-MicroPython 核心板搭配 DRV8701 双驱模块进行测试

# 示例程序运行效果为电机反复正反加减速转动
# C4 LED 会根据电机的正反转点亮或熄灭
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *

# 从 seekfree 库包含 MOTOR_CONTROLLER
from seekfree import MOTOR_CONTROLLER

# 包含 gc 与 time 类
import gc, time

# 延迟上电 避免 CR 时序控制导致屏幕还未成功启动
time.sleep_ms(100)

# 核心板上 C4 是 LED
# 学习板上 D9 对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 显示帮助信息
MOTOR_CONTROLLER.help()

# ------------------------------------------------------------------------------
#   构造接口 用于构建一个 MOTOR_CONTROLLER 对象
#   MOTOR_CONTROLLER_obj = MOTOR_CONTROLLER(index, freq, duty = 0, invert = False)
#       index           电机索引    |   必要参数 [  PWM_C30_DIR_C31, PWM_C28_DIR_C29, PWM_D4_DIR_D5, PWM_D6_DIR_D7,
#                                                   PWM_C30_PWM_C31, PWM_C28_PWM_C29, PWM_D4_PWM_D5, PWM_D6_PWM_D7]
#       freq            信号频率    |   必要参数 PWM 信号的频率 范围是 [1 - 100000]
#       duty            占空比值    |   可选参数 关键字参数 默认为 0 范围 ±10000 正数正转 负数反转 正转反转方向取决于 invert
#       invert          反向设置    |   可选参数 关键字参数 是否反向 默认为 False 可以通过这个参数调整电机方向极性
#       return          返回内容    |   正常情况下返回对应 MOTOR_CONTROLLER 的对象
# ------------------------------------------------------------------------------
motor_1 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = False)
motor_2 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = True)
motor_3 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5  , 13000, duty = 0, invert = False)
motor_4 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7  , 13000, duty = 0, invert = True)
# 本例程默认使用 DRV8701 双驱模块搭配双电机 ！！！
# 本例程默认使用 DRV8701 双驱模块搭配双电机 ！！！
# 本例程默认使用 DRV8701 双驱模块搭配双电机 ！！！

# MOTOR_CONTROLLER 接口 :
# ------------------------------------------------------------------------------
#   更新或获取占空比值
#   MOTOR_CONTROLLER.duty(duty)
#   duty = MOTOR_CONTROLLER.duty()
#       duty            占空比值    |   可选参数 填数值就设置新的占空比 否则返回当前占空比 范围是 ±10000
#       return          返回内容    |   当不输入 duty 参数时 返回当前的占空比值 否则无返回值
# ------------------------------------------------------------------------------
#   可以直接通过类调用 也可以通过对象调用 输出模块的使用帮助信息
#   MOTOR_CONTROLLER.help()
# ------------------------------------------------------------------------------
#   通过对象调用 输出当前对象的自身信息
#   MOTOR_CONTROLLER.info()
# ------------------------------------------------------------------------------

motor_1.info()
motor_2.info()
motor_3.info()
motor_4.info()

motor_dir = 1
motor_duty = 0
motor_duty_max = 1000

while True:
    time.sleep_ms(100)
    
    if motor_dir:
        motor_duty = motor_duty + 50
        if motor_duty >= motor_duty_max:
            motor_dir = 0
    else:
        motor_duty = motor_duty - 50
        if motor_duty <= -motor_duty_max:
            motor_dir = 1
    
    led.value(motor_duty < 0)
    # duty 接口更新占空比 范围 ±10000
    motor_1.duty(motor_duty)
    motor_2.duty(motor_duty)
    motor_3.duty(motor_duty)
    motor_4.duty(motor_duty)
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
