
# 本示例程序演示如何使用 machine 库的 Pin 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的蜂鸣器与拨码开关

# 示例程序运行效果为每 500ms(0.5s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 并且学习板上 D24 对应蜂鸣器每 500ms(0.5s) 响一次
# 并且实时输出 D8 引脚的电平
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 构造接口 是标准 MicroPython 的 machine.Pin 模块
#   Pin(pin, mode, [, pull = Pin.PULL_UP_47K, value = 1, drive = Pin.DRIVE_OFF])
#   pin     引脚名称    | 必要参数 引脚名称 本固件以核心板上引脚编号为准
#   mode    引脚模式    | 必要参数 对应引脚工作状态 Pin.x, x = {IN, OUT, OPEN_DRAIN}
#   pull    上拉下拉    | 可选参数 Pin.x, x = {PULL_UP, PULL_UP_47K, PULL_UP_22K, PULL_DOWN, PULL_HOLD}
#   value   初始电平    | 可选参数 关键字参数 可以设置为 {0, 1} 对应低电平与高电平
#   drive   内阻模式    | 可选参数 关键字参数 Pin.x, x = {PIN_DRIVE_OFF, PIN_DRIVE_0, ..., PIN_DRIVE_6}

# 核心板上 C4  是 LED
# 学习板上 D24 对应蜂鸣器
# 学习板上 D8  对应一号拨码开关
# 学习板上 D9  对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
beep    = Pin('D24', Pin.OUT, value = False)
switch1 = Pin('D8' , Pin.IN , pull = Pin.PULL_UP_47K)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state1  = switch1.value()
state2  = switch2.value()

# 其余接口：
# Pin.on()          # 端口电平置位
# Pin.off()         # 端口电平复位
# Pin.low()         # 端口电平输出低电平
# Pin.high()        # 端口电平输出高电平
# Pin.toggle()      # 端口电平翻转
# Pin.value(x)      # 传入参数 x 则将端口电平设置为对应 bool 值
#                   # 不传入参数则只返回端口电平 bool 值

led.value(False)
time.sleep_ms(100)
led.value(True)
time.sleep_ms(100)

while True:
    time.sleep_ms(50)
    beep.high()
    time.sleep_ms(50)
    beep.low()
    time.sleep_ms(400)

    led.toggle()
    print("Switch D8 is {:>1d}.".format(switch1.value()))
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
