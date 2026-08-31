
# 本示例程序演示如何使用 seekfree 库的 KEY_HANDLER 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的按键测试

# 示例程序运行效果为每 1000ms(1s) C4 LED 改变亮灭状态
# 当按键短按或者长按时通过 Type-C 的 CDC 虚拟串口输出信息
# 当 D9 引脚电平出现变化时退出测试程序

# KEY_HANDLER 的扫描周期计算方式
# Ticker 通过 start(y) 启动时 y 代表 Ticker 的周期
# 此时每 y 毫秒会触发一次 KEY_HANDLER 的更新
# 因此 KEY_HANDLER 的采集周期时间等于 y 本例程中就是 10ms

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 从 seekfree 库包含 KEY_HANDLER
from seekfree import KEY_HANDLER

# 包含 gc 与 time 类
import gc
import time

# 核心板上 C4 是 LED
# 学习板上 D9 对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 显示帮助信息
KEY_HANDLER.help()
time.sleep_ms(500)

# 构造接口 用于构建一个 KEY_HANDLER 对象
#   KEY_HANDLER(period)
#   period  扫描周期    |   必要参数 按键的扫描周期 一般配合填写 Tickter 的运行周期
key = KEY_HANDLER(10)

# 其余接口：
# KEY_HANDLER.capture()         # 执行一次按键状态扫描
# KEY_HANDLER.get()             # 输出当前四个按键状态
# KEY_HANDLER.clear([index])    # 清除按键状态 长按会锁定长按状态不被清除
#   index       按键序号    |   可选参数 1 - 4 清除对应按键的触发状态
# KEY_HANDLER.help()            # 可以直接通过类调用 也可以通过对象调用 输出模块的使用帮助信息
# KEY_HANDLER.info()            # 通过对象调用 输出当前对象的自身信息

key.info()
time.sleep_ms(500)

# 通过 get 接口读取数据
# 本质上是将 Python 对象与传感器数据缓冲区链接起来
# 所以只需要一次 KEY_HANDLER.get() 后就不需要再调用这个接口
# 之后直接使用获取的列表对象即可 它的数据会随 caputer 更新
key_data = key.get()

ticker_flag     = False
ticker_count    = 0
runtime_count   = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)

# 通过 capture 接口更新数据 但在这个例程中被 ticker.capture_list 模块接管了
# key.capture()
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  KEY_HANDLER, IMU660RX, IMU963RX, DL1X 和 TSL1401
pit1.capture_list(key)

# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(10)

while True:
    if (ticker_flag):
        # 按键数据为三个状态 0-无动作 1-短按 2-长按
        if key_data[0]:
            print("key1 = {:>6d}.".format(key_data[0]))
            key.clear(1)
        if key_data[1]:
            print("key2 = {:>6d}.".format(key_data[1]))
            key.clear(2)
        if key_data[2]:
            print("key3 = {:>6d}.".format(key_data[2]))
            key.clear(3)
        if key_data[3]:
            print("key4 = {:>6d}.".format(key_data[3]))
            key.clear(4)
        if (ticker_count % 100 == 0):
            led.toggle()
        ticker_flag = False
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        pit1.stop()
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
