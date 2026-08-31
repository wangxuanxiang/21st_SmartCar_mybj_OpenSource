
# 本示例程序用于测试核心板的引脚是否正常

# 使用 RT1021-MicroPython 核心板单独测试
# 使用 RT1021-MicroPython 核心板单独测试
# 使用 RT1021-MicroPython 核心板单独测试

# 不连接学习板或主板 不连接任何传感器
# 不连接学习板或主板 不连接任何传感器
# 不连接学习板或主板 不连接任何传感器

# 成功连接 Thonny 后运行本示例程序
# 如果核心板一切正常 那么控制台会输出 Test pass.
# 并且核心板 LED  C4 会呈现周期呼吸灯亮灭

# 如果有引脚对 GND 短路 那么会报错提示 "pin_name" state error. Can not pull to VCC.
# 如果有引脚对 VCC 短路 那么会报错提示 "pin_name" state error. Can not pull to GND.
# 如果有引脚相互连焊    那么会报错提示 "pin_name" and "pin_name" may shortcircuit.
# 报错时核心板 LED  C4 会快速连续闪烁

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 核心板上 C4 是 LED

# 调用 machine 库的 Pin 类实例化一个引脚对象
# 配置参数为 引脚名称 引脚方向 模式配置 默认电平
# 详细内容参考 固件接口说明
led = Pin('C4' , Pin.OUT, value = True)

# 定义所有待测引脚名称
pin_list = [
    "B3" ,  "B4" ,  "B5" ,
    "B8" ,  "B9" ,  "B10",  "B11",  "B12",  "B13",  "B14",  "B15",
    "B16",  "B17",  "B18",  "B19",  "B20",  "B21",  "B22",  "B23",
    "B24",  "B25",  "B26",  "B27",  "B28",  "B29",  "B30",  "B31",

    "C0" ,  "C1" ,  "C2" ,  "C3" ,          "C5" ,  "C6" ,  "C7" ,
    "C8" ,  "C9" ,  "C10",  "C11",  "C12",  "C13",  "C14",  "C15",
                    "C18",  "C19",  "C20",  "C21",  "C22",  "C23",
    "C24",  "C25",  "C26",  "C27",  "C28",  "C29",  "C30",  "C31",

    "D0" ,  "D1" ,  "D2" ,  "D3" ,  "D4" ,  "D5" ,  "D6" ,  "D7" ,
    "D8" ,  "D9" ,                          "D13",  "D14",  "D15",
    "D16",  "D17",  "D18",  "D19",  "D20",  "D21",  "D22",  "D23",
    "D24"]

LED_PERIOD          = 200
LED_TOGGLE_FACTOR   = 2

count               = 0
led_state           = 0

# 先将所有引脚设置为开漏输出高 避免原状态干扰
for pin_index in pin_list:
    pin_obj = Pin(pin_index, Pin.OPEN_DRAIN, pull = Pin.PULL_UP, value = True)
    del(pin_obj)

# 挨个检查开漏输出高时 每个引脚自身的电平状态 理论上什么都不接 上拉读取为高电平
# 如果读取到低电平 那么就证明有问题
for pin_index in pin_list:
    pin_obj = Pin(pin_index, Pin.OPEN_DRAIN, pull = Pin.PULL_UP, value = True)
    if pin_obj.value() != True:
        print(pin_index + " state error. Can not pull to VCC.\r\n")
        while True:
            time.sleep_ms(50)
            led.toggle()
    del(pin_obj)

# 挨个检查开漏输出低时 每个引脚自身的电平状态 理论上什么都不接 此时输出低读取就为低
# 如果读取到高电平 那么就证明有问题
for pin_index in pin_list:
    pin_obj = Pin(pin_index, Pin.OPEN_DRAIN, pull = Pin.PULL_UP, value = False)
    if pin_obj.value() != False:
        print(pin_index + " state error. Can not pull to GND.\r\n")
        while True:
            time.sleep_ms(50)
            led.toggle()
    del(pin_obj)

# 再将所有引脚设置为开漏输出高 避免原状态干扰
for pin_index in pin_list:
    pin_obj = Pin(pin_index, Pin.OPEN_DRAIN, pull = Pin.PULL_UP, value = True)
    del(pin_obj)

# 挨个检查开漏输出低时 其它引脚自身的电平状态 理论上其它引脚会维持之前的高电平
# 如果读取到低电平 那么就证明有问题
count = 0
while count < len(pin_list):
    pin_obj1 = Pin(pin_list[count], Pin.OPEN_DRAIN, pull = Pin.PULL_UP, value = False)
    for pin_index in pin_list:
        if(pin_index != pin_list[count]):
            pin_obj2 = Pin(pin_index, Pin.OPEN_DRAIN, pull = Pin.PULL_UP, value = True)
            if pin_obj2.value() != True:
                print(pin_list[count] + " and " + pin_index + " may shortcircuit.\r\n")
                while True:
                    time.sleep_ms(50)
                    led.toggle()
            del(pin_obj2)
    del(pin_obj1)
    count = count + 1

# 再将所有引脚设置为开漏输出高
for pin_index in pin_list:
    pin_obj = Pin(pin_index, Pin.OPEN_DRAIN, pull = Pin.PULL_UP, value = True)
    del(pin_obj)

# 测试通过 进入主程序呼吸灯
print("Test pass.\r\n")
while True:
    time.sleep_us(1)
    led.value(count > abs((LED_PERIOD - led_state / LED_TOGGLE_FACTOR)))
    count = (0) if((LED_PERIOD - 1) == count) else (count + 1)
    if (0 == count):
        led_state = (0) if((LED_PERIOD * 2 * LED_TOGGLE_FACTOR) <= led_state) else (led_state + 1)
