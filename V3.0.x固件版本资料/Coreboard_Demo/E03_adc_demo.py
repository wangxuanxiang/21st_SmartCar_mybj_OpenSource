
# 本示例程序演示如何使用 machine 库的 ADC 类接口
# 使用 RT1021-MicroPython 核心板
# 搭配对应拓展学习板的电池电压检测电路

# 示例程序运行效果为每 100ms(0.1s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 并通过 RT1021-MicroPython 核心板的 Type-C 的 CDC 虚拟串口输出一次转换数据结果
#   ADC value = xxxx, power voltage = xx.xxV.
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *

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
#   构造接口 是标准 MicroPython 的 machine.ADC 模块
#   ADC_obj = ADC(pin)
#       pin             引脚名称    |   必要参数 引脚名称 本固件以核心板上引脚编号为准
#       return          返回内容    |   正常情况下返回对应 ADC 通道的对象
# ------------------------------------------------------------------------------
power_adc = ADC('B27')
# 学习板上 B27 对应输入电压分压检测电路

# ADC 接口 :
# ------------------------------------------------------------------------------
#   读取当前端口的 ADC 转换值
#   ADC.read_u16()
#       return          返回内容    |   ADC 转换数值 数据返回范围是 [0, 65535]
# ------------------------------------------------------------------------------

while True:
    # 延时 100 ms
    time.sleep_ms(100)
    # 翻转 C4 LED 电平
    led.toggle()
    
    power_adc_value = power_adc.read_u16()
    # 学习板上分压电路为 1/(1 + 10) 参考电压 3.3V
    # 因此换算公式为 power_adc_value / 65535 * 3.3 * 11 = power_voltage (V)
    print("ADC value = {:>6d}, power voltage = {:>2.2f}V.\r\n".format(
        power_adc_value,
        power_adc_value / 65535 * 3.3 * 11))
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
