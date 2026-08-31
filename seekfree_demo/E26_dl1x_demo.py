
# 本示例程序演示如何使用 seekfree 库的 DL1X 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板与 DL1X 模块测试
# 当 D9 引脚电平出现变化时退出测试程序

# 示例程序运行效果为每 200ms(0.2s) 通过 Type-C 的 CDC 虚拟串口输出信息
# 可以通过 C19 的电平状态来控制是否退出测试程序
# 如果看到 Thonny Shell 控制台输出 ValueError: Module init fault. 报错
# 就证明 DL1X 模块连接异常 或者模块型号不对 或者模块损坏
# 请检查模块型号是否正确 接线是否正常 线路是否导通 无法解决时请联系技术支持

# DL1X 的更新周期计算方式
# DL1X 通过 DL1X(x) 初始化构建对象时 传入的 x 代表采集分频数
# 也就是需要进行几次 caputer 触发才会更新一次数据
# 当触发次数大于等于 x 时 DL1X 才会更新一次数据
# Ticker 通过 Ticker.start(y) 启动时 y 代表 Ticker 的周期
# 当通过 Ticker.capture_list() 将 IMU 与 Ticker 关联后
# 此时每 y 毫秒会进行一次 DL1X 的 caputer 触发
# 因此 DL1X 的数据更新周期等于 y * x
# 本例程中就是 50ms * 1 = 50ms

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 从 seekfree 库包含 DL1X
from seekfree import DL1X

# 包含 gc 与 time 类
import gc
import time

# 学习板上 D9 对应二号拨码开关
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 显示帮助信息
# 帮助信息中会显示支持那些模块
# 以及具体的 刷新频率 / 量程范围
DL1X.help()
time.sleep_ms(500)

# 构造接口 用于构建一个 DL1X 对象
#   DL1X([capture_div])
#   capture_div 采集分频    |   非必要参数 默认为 1 也就是每次都采集 代表多少次触发进行一次采集
tof = DL1X()

# 其余接口：
# DL1X.capture()    # 执行一次 DL1X 数据采集触发 达到触发数时执行采集并将数据缓存
# DL1X.get()        # 输出当前采集缓存的 DL1X 数据
# DL1X.read()       # 立即进行一次 capture 并输出缓存数据
# DL1X.help()       # 可以直接通过类调用 也可以通过对象调用 输出模块的使用帮助信息
# DL1X.info()       # 通过对象调用 输出当前对象的自身信息

tof.info()
time.sleep_ms(500)

ticker_flag = False
ticker_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 50) if (ticker_count < 1000) else (50)

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)

# 通过 capture 接口更新数据 但在这个例程中被 ticker.capture_list 模块接管了
# tof.capture()
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  KEY_HANDLER, IMU660RX, IMU963RX, DL1X 和 TSL1401
pit1.capture_list(tof)

# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(50)

while True:
    if (ticker_flag and ticker_count % 200 == 0):
        # 通过 get 接口读取数据
        tof_data = tof.get()
        print("distance = {:>6d}.".format(tof_data))
        ticker_flag = False
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
