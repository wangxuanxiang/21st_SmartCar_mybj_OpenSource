
# 本示例程序演示如何使用 seekfree 库的 TSL1401 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板与 TSL1401 IPS200 模块测试

# 示例程序运行效果是实时在 IPS200 屏幕上显示 TSL1401 的采集图像
# 当 D9 引脚电平出现变化时退出测试程序

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

# 包含 display 库
from display import *

# 从 seekfree 库包含 TSL1401
from seekfree import TSL1401

# 包含 gc 与 time 类
import gc
import time

# 核心板上 C4 是 LED
# 学习板上 D9 对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 定义片选引脚 拉高拉低一次 CS 片选确保屏幕通信时序正常
cs = Pin('B29' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
cs.high()
cs.low()
# 定义控制引脚
rst = Pin('B31', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc  = Pin('B5' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C21', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 新建 LCD 驱动实例 这里的索引范围与 SPI 示例一致 当前仅支持 IPS200
drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
# 新建 LCD 实例
lcd = LCD(drv)
# color 接口设置屏幕显示颜色 {前景色,背景色}
lcd.color(0xFFFF, 0x0000)
# mode 接口设置屏幕显示模式 {0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转}
lcd.mode(2)
# 清屏
lcd.clear(0x0000)

# 显示帮助信息
TSL1401.help()
time.sleep_ms(500)

# 构造接口 用于构建一个 TSL1401 对象
#   TSL1401([capture_div])
#   capture_div 采集分频    |   非必要参数 默认为 1 也就是每次都采集 代表多少次触发进行一次采集
ccd = TSL1401(10)

# 其余接口：
# TSL1401.set_resolution(resolution)    # 设置 TSL1401 的转换精度
#   resolution  接口索引    |   必要参数 TSL1401.x , x = {RES_8BIT, RES_12BIT}
# TSL1401.capture()                     # 执行一次 TSL1401 数据采集触发 达到触发数时执行采集并将数据缓存
# TSL1401.get()                         # 输出当前采集缓存的 TSL1401 数据
# TSL1401.read()                        # 立即进行一次 capture 并输出缓存数据
# TSL1401.help()                        # 可以直接通过类调用 也可以通过对象调用 输出模块的使用帮助信息
# TSL1401.info()                        # 通过对象调用 输出当前对象的自身信息

ccd.info()
time.sleep_ms(500)

# 调整 CCD 的采样精度为 12bit 数值范围 [0, 4095]
ccd.set_resolution(TSL1401.RES_12BIT)

ccd.info()
time.sleep_ms(500)

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

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)
    if(0 == (ticker_count % 10)):
        # 翻转 C4 LED 电平
        led.toggle()

# 实例化 PIT ticker 模块 参数为编号 [0, 3] 最多四个
pit1 = ticker(1)

# 通过 capture 接口更新数据 但在这个例程中被 ticker.capture_list 模块接管了
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
        # 通过 wave 接口显示数据波形 (x,y,width,high,data,data_max)
        # x - 起始显示 X 坐标
        # y - 起始显示 Y 坐标
        # width - 数据显示宽度 等同于数据个数
        # high - 数据显示高度
        # data - 数据对象 这里基本仅适配 TSL1401 的 get 接口返回的数据对象
        # max - 数据最大值  TSL1401.RES_8BIT  的数据范围 [0, 255 ]
        #                   TSL1401.RES_12BIT 的数据范围 [0, 4095]
        lcd.wave(0,  0, 128, 64, ccd_data1, max = 4095)
        lcd.wave(0, 64, 128, 64, ccd_data2, max = 4095)
        lcd.wave(0,128, 128, 64, ccd_data3, max = 4095)
        lcd.wave(0,192, 128, 64, ccd_data4, max = 4095)
        
        ticker_flag = False
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
