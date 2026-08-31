
# 本示例程序演示如何使用 machine 库的 Pin 类接口的外部中断
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的霍尔停车检测接口

# 示例程序运行效果为每 1000ms(1s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 当 D19 引脚电平出现由高拉低的下降沿时 触发一次外部中断回调
# 中断回调通过 RT1021-MicroPython 核心板的 Type-C 的 CDC 虚拟串口输出触发次数
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 核心板上 C4 是 LED
# 学习板上 D19 对应霍尔停车检测接口
# 学习板上 D9  对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
hall    = Pin('D19', Pin.IN , pull = Pin.PULL_UP_47K)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 新建一个变量用于计数
hall_count = 0

# 定义一个回调函数 必须有一个参数用于传递实例本身
def hall_handler(x):
    # global 是全局变量修饰 说明这里使用的是一个全局变量
    # 否则函数会新建一个临时变量
    global hall_count

    # 计数递增 并通过 Type-C 的 CDC 虚拟串口输出触发次数
    hall_count = hall_count + 1
    print("hall_count ={:>6d}, hall_state ={:>6d}".format(hall_count, x.value()))

# 配置 Pin 的中断 也就是外部中断 EXTI
#   Pin.irq(handler, trigger, hard)
#   handler 回调函数    | 必要参数 触发后对应的回调函数 python 函数
#   trigger 触发模式    | 必要参数 可用值为 Pin.x, x = {IRQ_RISING, IRQ_FALLING}
#   hard    应用模式    | 可选参数 可用值为 False True
hall.irq(hall_handler, Pin.IRQ_FALLING, False)

while True:
    time.sleep_ms(1000)
    led.toggle()
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
