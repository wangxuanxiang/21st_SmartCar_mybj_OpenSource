
# 本示例程序演示如何使用 smartcar 库的 ticker 类接口
# 使用 RT1021-MicroPython 核心板即可测试

# 示例程序运行效果为每 100ms(0.1s) 通过 Type-C 的 CDC 虚拟串口输出一次信息
# 并改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 包含 gc 类
import gc

# 核心板上 C4 是 LED
# 学习板上 D9 对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

ticker_flag = False
ticker_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    ticker_flag = True  # 否则它会新建一个局部变量

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(100)

while True:
    if (ticker_flag):
        ticker_flag = False
        ticker_count = ticker_count + 1
        
        # 翻转 C4 LED 电平
        led.toggle()
        print("Ticker trigger {:>6d}.".format(ticker_count))
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        pit1.stop()
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
