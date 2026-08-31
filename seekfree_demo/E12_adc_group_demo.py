
# 本示例程序演示如何使用 smartcar 库的 ADC_Group 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的四路电磁运放接口

# 示例程序运行效果为每 200ms(0.2s) C4 LED 改变亮灭状态
# 并通过 Type-C 的 CDC 虚拟串口输出一次信息
# 当 D9 引脚电平出现变化时退出测试程序

# ADC_Group 的采集周期计算方式
# Ticker 通过 start(y) 启动时 y 代表 Ticker 的周期
# 此时每 y 毫秒会触发一次 ADC_Group 的更新
# 因此 ADC_Group 的采集周期时间等于 y 本例程中就是 10ms

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker ADC_Group
from smartcar import ticker
from smartcar import ADC_Group

# 包含 gc 类
import gc

# 核心板上 C4 是 LED
# 学习板上 D9 对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 构造接口 用于构建一个 ADC_Group 对象
#   ADC_Group(id)
#   id      模块索引    |   必要参数 RT1021 对应有 [1,2] 总共两个模块可选
adc_group = ADC_Group(1)
# 将四个通道添加进来
adc_group.addch('B12')
adc_group.addch('B14')
adc_group.addch('B15')
adc_group.addch('B17')

# 其余接口：
# ADC_Group.init(id, period = ADC_Group.PMODE3, average = ADC_Group.AVG16)
#   id      模块索引    |   必要参数 RT1021 对应有 [1,2] 总共两个模块可选
#   period  采样周期    |   可选参数 关键字输入 默认 ADC_Group.x, x = {PMODE0, PMODE1, PMODE2, PMODE3}
#   average 均值选项    |   可选参数 关键字输入 默认 ADC_Group.x, x = {AVG1, AVG4, AVG8, AVG16, AVG32}
# ADC_Group.capture()       # 触发一次 ADC_Group 的转换
# ADC_Group.get()           # 将 ADC_Group 转换结果更新到数据缓冲区 并返回为一个列表
# ADC_Group.read()          # 触发一次转换 并将转换结果更新到数据缓冲区 返回为一个列表

ticker_flag = False
ticker_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)

# 通过 capture 接口更新数据 但在这个例程中被 ticker.capture_list 模块接管了
# adc_group.capture()
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  KEY_HANDLER, IMU660RX, IMU963RX, DL1X 和 TSL1401
pit1.capture_list(adc_group)

# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(10)

while True:
    if (ticker_flag and ticker_count % 20 == 0):
        led.toggle()
        # 通过 get 接口获取数据 数据返回范围是 0-4095
        adc_data = adc_group.get()
        print("adc={:>6d},{:>6d},{:>6d},{:>6d}.\r\n".format(
            adc_data[0], adc_data[1], adc_data[2], adc_data[3]))
        ticker_flag = False

    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        pit1.stop()
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
