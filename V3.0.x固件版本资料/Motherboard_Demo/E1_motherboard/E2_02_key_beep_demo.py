
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
import gc, time

# 延迟上电 避免 CR 时序控制导致屏幕还未成功启动
time.sleep_ms(100)

# 核心板上 C4 是 LED
# 学习板上 D24 对应蜂鸣器
# 学习板上 D9 对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
beep    = Pin('D24', Pin.OUT, value = False)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 显示帮助信息
KEY_HANDLER.help()

# ------------------------------------------------------------------------------
#   构造接口 用于构建一个 KEY_HANDLER 对象
#   KEY_HANDLER_obj = KEY_HANDLER(period)
#       period          扫描周期    |   必要参数 按键的扫描周期 毫秒单位 一般配合填写 Tickter 的运行周期
#       return          返回内容    |   正常情况下返回对应 KEY_HANDLER 的对象
# ------------------------------------------------------------------------------
key = KEY_HANDLER(10)

# KEY_HANDLER 接口 :
# ------------------------------------------------------------------------------
#   执行一次按键状态扫描
#   KEY_HANDLER.capture()
# ------------------------------------------------------------------------------
#   获取当前四个按键状态 只需要一次 KEY_HANDLER.get() 后就不需要再调用这个接口
#   key_state = KEY_HANDLER.get()
#       return          返回内容    |   返回当前四个按键状态 返回为一个列表
# ------------------------------------------------------------------------------
#   触发一次 capture 方法接口 并获取当前四个按键状态
#   key_state = KEY_HANDLER.read()
#       return          返回内容    |   返回当前四个按键状态 返回为一个列表
# ------------------------------------------------------------------------------
#   清除按键状态 长按会锁定长按状态不被清除
#   KEY_HANDLER.clear(index)
#   KEY_HANDLER.clear()
#       index           按键序号    |   可选参数 [1, 4] 清除对应按键的触发状态 不输入参数则清空所有按键状态
# ------------------------------------------------------------------------------
#   获取当前按键实例的周期设置
#   period = KEY_HANDLER.get_period()
#       return          返回内容    |   返回当前按键实例的周期 毫秒单位
# ------------------------------------------------------------------------------
#   可以直接通过类调用 也可以通过对象调用 输出模块的使用帮助信息
#   KEY_HANDLER.help()
# ------------------------------------------------------------------------------
#   通过对象调用 输出当前对象的自身信息
#   KEY_HANDLER.info()
# ------------------------------------------------------------------------------

key.info()

# 通过 get 接口读取数据
# 本质上是将 Python 对象与传感器数据缓冲区链接起来
# 所以只需要一次 KEY_HANDLER.get() 后就不需要再调用这个接口
# 之后直接使用获取的列表对象即可 它的数据会随 caputer 更新
key_data = key.get()

ticker_flag     = False
ticker_count    = 0
runtime_count   = 0
beep_flag       = 0

# 定义一个回调函数 必须有一个参数用于传递实例本身 这个参数就是 ticker 实例自身
def time_pit_handler (ticker_obj):
    # 需要注意的是这里得使用 global 修饰全局属性
    # 否则它会新建一个局部变量
    global ticker_flag
    global ticker_count
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

pit1 = ticker(1)
pit1.capture_list(key)
pit1.callback(time_pit_handler)
pit1.start(10)
while True:
    if (ticker_flag):
        # Tips : 通过 capture 接口更新数据 但在这个例程中被 ticker.capture_list 模块接管了
        # key.capture()

        # 按键数据为三个状态 0-无动作 1-短按 2-长按
        if key_data[0]:
            beep_flag = 1 - beep_flag
            print("key1 = {:>6d}.".format(key_data[0]))
            key.clear(1)
        if key_data[1]:
            beep_flag = 1 - beep_flag
            print("key2 = {:>6d}.".format(key_data[1]))
            key.clear(2)
        if key_data[2]:
            beep_flag = 1 - beep_flag
            print("key3 = {:>6d}.".format(key_data[2]))
            key.clear(3)
        if key_data[3]:
            beep_flag = 1 - beep_flag
            print("key4 = {:>6d}.".format(key_data[3]))
            key.clear(4)
        if (ticker_count % 10 == 0):
            if beep_flag:
                beep.toggle()
            else:
                beep.low()
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
