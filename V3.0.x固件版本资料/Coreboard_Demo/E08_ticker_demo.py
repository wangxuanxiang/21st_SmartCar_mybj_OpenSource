
# 本示例程序演示如何使用 smartcar 库的 ticker 类接口
# 使用 RT1021-MicroPython 核心板即可测试

# 示例程序运行效果为每 100ms(0.1s) 通过 Type-C 的 CDC 虚拟串口输出一次信息
# 并改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 包含 gc 与 time 类
import gc, time

# 延迟上电 避免 CR 时序控制导致屏幕还未成功启动
time.sleep_ms(100)

# 核心板上 C4 是 LED
# 学习板上 D9 对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# ------------------------------------------------------------------------------
#   构造接口 实例化 PIT ticker 模块
#   ticker_obj = ticker(id)
#       id              接口编号    |   必要参数 本固件支持 [0-3] 最多四个
#       return          返回内容    |   正常情况下返回对应 ticker 的对象
# ------------------------------------------------------------------------------
pit1 = ticker(1)

# ticker 接口 :
# ------------------------------------------------------------------------------
#   关联回调函数
#   ticker.callback(handler)
#       handler         回调函数    |   必要参数 带自身对象传递的回调函数
# ------------------------------------------------------------------------------
#   以指定周期启动 ticker
#   ticker.start(period_ms)
#       period_ms       周期时间    |   必要参数 周期 毫秒单位 建议不低于 5ms
# ------------------------------------------------------------------------------
#   停止 ticker
#   ticker.stop()
# ------------------------------------------------------------------------------
#   绑定捕获对象 最少一个最多八个 绑定后每个回调前都会先调用对象的 capture 方法接口
#   ticker.capture_list(sensor_obj, ...)
#       sensor_obj      捕获对象    |   必要参数 Sensor 框架的传感器对象 最少一个 最多八个 (imu, ccd, key...)
#                                       可关联 smartcar 的 ADC_Group_x 与 encoder_x
#                                       可关联 seekfree 的  KEY_HANDLER, IMU660RX, IMU963RX, DL1X 和 TSL1401
# ------------------------------------------------------------------------------
#   获取计数值
#   count = ticker.ticks()
#       return          返回内容    |   返回当前 ticker 的计数数值
# ------------------------------------------------------------------------------

ticker_flag = False
ticker_count = 0

# 定义一个回调函数 必须有一个参数用于传递实例本身 这个参数就是 ticker 实例自身
def time_pit_handler (ticker_obj):
    # 需要注意的是这里得使用 global 修饰全局属性
    # 否则它会新建一个局部变量
    global ticker_flag
    ticker_flag = True

pit1.callback(time_pit_handler)
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
