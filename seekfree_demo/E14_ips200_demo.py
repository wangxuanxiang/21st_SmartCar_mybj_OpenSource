
# 本示例程序演示如何使用 display 库
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的屏幕接口测试

# 示例程序运行效果为循环在 IPS200 上刷新全屏颜色 然后显示数字 画线
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *
from array import *

# 包含 display 库
from display import *

# 包含 gc time 类
import gc
import time

# 学习板上 D9 对应二号拨码开关
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 定义片选引脚 拉高拉低一次 CS 片选确保屏幕通信时序正常
cs = Pin('B29' , Pin.OUT, value=True)
cs.high()
cs.low()

# 定义控制引脚
rst = Pin('B31', Pin.OUT, value=True)
dc  = Pin('B5' , Pin.OUT, value=True)
blk = Pin('C21', Pin.OUT, value=True)

# 构造接口 用于构建一个 LCD_Drv 对象
#   LCD_Drv(SPI_INDEX, BAUDRATE, DC_PIN, RST_PIN, LCD_TYPE)
#   SPI_INDEX   接口索引    |   必要参数 关键字输入 选择屏幕所用的 SPI 接口索引
#   BAUDRATE    通信速率    |   必要参数 关键字输入 SPI 的通信速率 最高 60MHz
#   DC_PIN      命令引脚    |   必要参数 关键字输入 一个 Pin 实例
#   RST_PIN     复位引脚    |   必要参数 关键字输入 一个 Pin 实例
#   LCD_TYPE    屏幕类型    |   必要参数 关键字输入 目前仅支持 LCD_Drv.LCD200_TYPE
drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)

# 构造接口 用于构建一个 LCD 对象
#   LCD(LCD_Drv)
#   LCD_Drv     接口对象    |   必要参数 LCD_Drv 对象
lcd = LCD(drv)

# 修改 LCD 的前景色与背景色
#   LCD.color(pcolor, bgcolor)
#   pcolor      前景色     |   必要参数 RGB565 格式
#   bgcolor     背景色     |   必要参数 RGB565 格式
lcd.color(0xFFFF, 0x0000)

# 修改 LCD 的显示方向
#   LCD.mode(dir)
#   dir         显示方向    |   必要参数 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
lcd.mode(2)

# 清屏 不传入参数就使用当前的 背景色 清屏
#   LCD.clear([color])
#   color       颜色数值    |   非必要参数 RGB565 格式 输入参数则更新背景色并清屏
lcd.clear(0x0000)

arr = array('h', [0] * 128)
for i in range(0, 64):
    arr[i] = i * 64
    arr[127 - i] = i * 64

while True:
    time.sleep_ms(500)
    lcd.clear(0xF800)
    time.sleep_ms(500)
    lcd.clear(0x07E0)
    time.sleep_ms(500)
    lcd.clear(0x001F)
    time.sleep_ms(500)
    lcd.clear(0xFFFF)
    time.sleep_ms(500)
    lcd.clear(0x0000)
    time.sleep_ms(500)

    # 不管你要显示 字符 还是数字
    # 对于 Python 来说他们都是一样的
    # 用 format 或者 "%..."%(...) 统一处理为字符串对象
    
    # 显示数据与显示字符串对于 Python 来说没有区别
    # 显示字符串的函数 [x,y,str,color]
    # x - 起始显示 X 坐标
    # y - 起始显示 Y 坐标
    # str - 字符串
    # color - 字符颜色 可以不填使用默认的前景色
    lcd.str12(0,  0, "15={:b},{:d},{:o},{:#x}.".format(15,15,15,15), 0xF800)
    lcd.str16(0, 12, "1.234={:>.2f}.".format(1.234), 0x07E0)
    lcd.str24(0, 28, "123={:<6d}.".format(123), 0x001F)
    lcd.str32(0, 52, "123={:>6d}.".format(123), 0xFFFF)
    
    # 显示一条线的函数 [x1,y1,x1,y1,color,thick]
    # x1 - 起始 X 坐标
    # y1 - 起始 Y 坐标
    # x2 - 结束 X 坐标
    # y2 - 结束 Y 坐标
    # color - 线的颜色 可以不填使用默认的前景色
    # thick - 线的宽度 可以不填 默认 1
    lcd.line(  0, 84, 200, 16 + 84, color = 0xFFFF, thick = 1)
    lcd.line(200, 84,   0, 16 + 84, color = 0x3616, thick = 3)
    
    # 通过 wave 接口显示数据波形 (x,y,width,high,data,data_max)
    # x - 起始显示 X 坐标
    # y - 起始显示 Y 坐标
    # width - 数据显示宽度 等同于数据个数
    # high - 数据显示高度
    # data - 数据对象 这里基本仅适配 TSL1401 的 get 接口返回的数据对象
    # max - 数据最大值  TSL1401.RES_8BIT  的数据范围 [0, 255 ]
    #                   TSL1401.RES_12BIT 的数据范围 [0, 4095]
    lcd.wave(0, 120, 128, 64, arr, max = 4095)
    lcd.wave(0, 200, 64, 32, arr, max = 4095)
    time.sleep_ms(1000)
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
