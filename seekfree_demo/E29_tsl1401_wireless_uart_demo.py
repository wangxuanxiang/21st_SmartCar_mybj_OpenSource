
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

# 包含 gc time 类
import gc
import time

# 学习板上 D9 对应二号拨码开关
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 构造接口 用于构建一个 TSL1401 对象
#   TSL1401([capture_div])
#   capture_div 采集分频    |   非必要参数 默认为 1 也就是每次都采集 代表多少次触发进行一次采集
ccd = TSL1401(10)
# 调整 CCD 的采样精度为 12bit 数值范围 [0, 4095]
ccd.set_resolution(TSL1401.RES_12BIT)

# 构造接口 用于构建一个 WIRELESS_UART 对象
#   WIRELESS_UART([baudrate])
#   baudrate    波特率 |   可选参数 默认 460800
wireless = WIRELESS_UART(460800)

ticker_flag = False
ticker_count = 0
runtime_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)

# 通过 capture 接口更新数据 但在这个例程中被 ticker 模块接管了
# ccd.capture()
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  KEY_HANDLER, IMU660RX, IMU963RX, DL1X 和 TSL1401
pit1.capture_list(ccd)

# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(10)

while True:
    if (ticker_flag):
        # 通过 get 接口读取数据 参数 [0,1,2,3] 对应学习板上 CCD1/2/3/4 接口
        ccd_data1 = ccd.get(0)
        ccd_data2 = ccd.get(1)
        ccd_data3 = ccd.get(2)
        ccd_data4 = ccd.get(3)
        
        # send_ccd_image 将对应编号的 CCD 数据上传到逐飞助手
        # 可选参数共有六个 WIRELESS_UART.
        #   [CCD1_BUFFER_INDEX  ,   CCD2_BUFFER_INDEX   ]
        #   [CCD3_BUFFER_INDEX  ,   CCD4_BUFFER_INDEX   ]
        #   [CCD1_2_BUFFER_INDEX,   CCD3_4_BUFFER_INDEX ]
        # 分别代表 仅显示 CCD1 图像 、 仅显示 CCD2 图像 、 两个 CCD 图像一起显示
        wireless.send_ccd_image(WIRELESS_UART.CCD1_2_BUFFER_INDEX)
        
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
