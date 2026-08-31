
# 本示例程序演示如何使用 smartcar 库的 encoder 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的编码器接口测试

# 示例程序运行效果为每 200ms(0.2s) C4 LED 改变亮灭状态
# 并通过 Type-C 的 CDC 虚拟串口输出一次信息
# 当 D9 引脚电平出现变化时退出测试程序

# encoder 的采集周期计算方式
# Ticker 通过 start(y) 启动时 y 代表 Ticker 的周期
# 此时每 y 毫秒会触发一次 encoder.capture() 的更新
# encoder 实例化时输入的 capture_div 参数代表触发采集的分频
# 意思是触发几次 encoder.capture() 后实际进行一次脉冲数据采集更新到数据缓冲区
# 假设 encoder 实例化 capture_div = 5 然后 Ticker.start(10)
# 那么此时 encoder 的采集周期就是 10ms * 5 = 50ms

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker encoder
from smartcar import ticker
from smartcar import encoder

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
#   构造接口 用于构建一个 encoder 对象
#   encoder_obj = encoder(PhaseA, PhaseB, invert = False, capture_div = 1)
#       PhaseA          引脚名称    |   必要参数 引脚名称字符串 编码器 A 相或 PLUS 引脚
#       PhaseB          引脚名称    |   必要参数 引脚名称字符串 编码器 B 相或 DIR  引脚
#       invert          模块索引    |   可选参数 是否反向 可以通过这个参数调整编码器旋转方向数据极性
#       capture_div     模块索引    |   可选参数 关键字输入 设置采集触发分频
#       return          返回内容    |   正常情况下返回对应 encoder 的对象
# ------------------------------------------------------------------------------
encoder_1 = encoder("D15", "D16", True, capture_div = 10)
encoder_2 = encoder("D13", "D14", capture_div = 5)
encoder_3 = encoder("C2" , "C3" , True)
encoder_4 = encoder("C0" , "C1" )
# 对应学习板的编码器接口 1/2/3/4

# encoder 接口 :
# ------------------------------------------------------------------------------
#   增加一个 encoder 的采集请求 当达到 capture_div 数量时进行一次脉冲数据采集更新
#   encoder.capture()
# ------------------------------------------------------------------------------
#   从 encoder 数据缓冲区获取最新的脉冲数据
#   count = encoder.get()
#       return          返回内容    |   返回当前 encoder 的计数数值
# ------------------------------------------------------------------------------
#   增加一个 encoder 的采集请求 并从 encoder 数据缓冲区获取最新的脉冲数据
#   count = encoder.read()
#       return          返回内容    |   返回当前 encoder 的计数数值
# ------------------------------------------------------------------------------

ticker_flag = False
ticker_count = 0

# 定义一个回调函数 必须有一个参数用于传递实例本身 这个参数就是 ticker 实例自身
def time_pit_handler (ticker_obj):
    # 需要注意的是这里得使用 global 修饰全局属性
    # 否则它会新建一个局部变量
    global ticker_flag, ticker_count
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

pit1 = ticker(1)
pit1.capture_list(encoder_1, encoder_2, encoder_3, encoder_4)
pit1.callback(time_pit_handler)
pit1.start(10)

while True:
    if (ticker_flag and ticker_count % 20 == 0):
        led.toggle()
        # Tips : 通过 capture 接口更新数据 但在这个例程中被 ticker.capture_list 模块接管了
        # encoder_1.capture()
        # encoder_2.capture()
        # encoder_3.capture()
        # encoder_4.capture()

        # 通过 get 接口读取数据
        enc1_data = encoder_1.get()
        enc2_data = encoder_2.get()
        enc3_data = encoder_3.get()
        enc4_data = encoder_4.get()
        print("enc = %6d, %6d, %6d, %6d.\r\n"%(enc1_data, enc2_data, enc3_data, enc4_data))
        ticker_flag = False
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        pit1.stop()
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
