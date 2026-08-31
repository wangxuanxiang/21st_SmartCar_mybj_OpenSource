
# 本示例程序演示如何使用 seekfree 库的 WIRELESS_UART 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板与无线串口模块测试
# 当 D9 引脚电平出现变化时退出测试程序

# 示例程序运行效果是通过无线串口模块接收逐飞助手下发的调参数据
# 显示在 Thonny Shell 控制台并发送回逐飞助手的虚拟示波器显示
# 如果看到 Thonny Shell 控制台输出 ValueError: Module init fault. 报错
# 就证明 无线串口 模块连接异常 或者模块型号不对 或者模块损坏
# 请检查模块型号是否正确 接线是否正常 线路是否导通 无法解决时请联系技术支持

# TSL1401 的曝光计算方式
# TSL1401 通过 TSL1401(x) 初始化构建对象时 传入的 x 代表采集分频数
# 也就是需要进行几次 caputer 触发才会更新一次数据
# 当触发次数大于等于 x 时 TSL1401 才会更新一次数据
# Ticker 通过 Ticker.start(y) 启动时 y 代表 Ticker 的周期
# 当通过 Ticker.capture_list() 将 TSL1401 与 Ticker 关联后
# 此时每 y 毫秒会进行一次 TSL1401 的 caputer 触发
# 因此 TSL1401 的数据更新周期等于 y * x
# 本例程中就是 10ms * 10 = 100ms

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 从 seekfree 库包含 TSL1401 / WIRELESS_UART
from seekfree import TSL1401
from seekfree import WIRELESS_UART

# 包含 gc 与 time 类
import gc, time

# 延迟上电 避免 CR 时序控制导致屏幕还未成功启动
time.sleep_ms(100)

# 学习板上 D9 对应二号拨码开关
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 显示帮助信息
TSL1401.help()

# ------------------------------------------------------------------------------
#   构造接口 用于构建一个 TSL1401 对象
#   TSL1401_obj = TSL1401(capture_div)
#       capture_div     采集分频    |   非必要参数 默认为 1 也就是每次都采集 代表多少次触发进行一次采集
#       return          返回内容    |   正常情况下返回对应 TSL1401 的对象
# ------------------------------------------------------------------------------
ccd = TSL1401(10)

# TSL1401 接口 :
# ------------------------------------------------------------------------------
#   设置 TSL1401 的转换精度
#   TSL1401.set_resolution(resolution)
#       resolution      精度参数    |   必要参数 TSL1401.x , x = {RES_8BIT, RES_12BIT}
# ------------------------------------------------------------------------------
#   增加一个 TSL1401 的采集请求 当达到 capture_div 数量时进行一次采集并将数据缓存
#   TSL1401.capture()
# ------------------------------------------------------------------------------
#   从 TSL1401 数据缓冲区获取最新的数据
#   data_buffer = TSL1401.get()
#       return          返回内容    |   返回当前 TSL1401 的转换数值 为一个列表
# ------------------------------------------------------------------------------
#   立即进行一次 capture 并从 TSL1401 数据缓冲区获取最新的数据
#   data_buffer = TSL1401.read()
#       return          返回内容    |   返回当前 TSL1401 的转换数值 为一个列表
# ------------------------------------------------------------------------------
#   可以直接通过类调用 也可以通过对象调用 输出模块的使用帮助信息
#   TSL1401.help()
# ------------------------------------------------------------------------------
#   通过对象调用 输出当前对象的自身信息
#   TSL1401.info()
# ------------------------------------------------------------------------------

ccd.set_resolution(TSL1401.RES_12BIT)
ccd.info()

# 通过 get 接口读取数据 参数 [0, 3] 对应学习板上 CCD1/2/3/4 接口
# 本质上是将 Python 对象与传感器数据缓冲区链接起来
# 所以只需要一次 TSL1401.get() 后就不需要再调用这个接口
# 之后直接使用获取的列表对象即可 它的数据会随 caputer 更新
ccd_data1 = ccd.get(0)
ccd_data2 = ccd.get(1)
ccd_data3 = ccd.get(2)
ccd_data4 = ccd.get(3)

ticker_flag = False
ticker_count = 0
runtime_count = 0

# 定义一个回调函数 必须有一个参数用于传递实例本身 这个参数就是 ticker 实例自身
def time_pit_handler (ticker_obj):
    # 需要注意的是这里得使用 global 修饰全局属性
    # 否则它会新建一个局部变量
    global ticker_flag
    global ticker_count
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

pit1 = ticker(1)
pit1.capture_list(ccd)
pit1.callback(time_pit_handler)
pit1.start(10)

wireless = WIRELESS_UART(460800)

while True:
    if (ticker_flag):
        # Tips : 通过 capture 接口更新数据 但在这个例程中被 ticker.capture_list 模块接管了
        # ccd.capture()
        
        wireless.send_ccd_image(WIRELESS_UART.CCD1_BUFFER_INDEX, 0x0000)
        wireless.send_ccd_image(WIRELESS_UART.CCD2_BUFFER_INDEX, 0xF800)
        wireless.send_ccd_image(WIRELESS_UART.CCD3_BUFFER_INDEX, 0x07E0)
        wireless.send_ccd_image(WIRELESS_UART.CCD4_BUFFER_INDEX, 0x001F)
        
        ticker_flag = False
        runtime_count = runtime_count + 1
        if(0 == runtime_count % 100):
            print("runtime_count = {:>6d}.".format(runtime_count))
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
