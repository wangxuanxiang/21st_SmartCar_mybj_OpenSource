
# 本示例程序演示如何使用 machine 库的 IIC 类接口
# 使用 RT1021-MicroPython 核心板
# 可以接入传感器测试

# 示例程序运行效果为每 500ms(0.5s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 并通过对应引脚进行 IIC 数据传输
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 核心板上 C4 是 LED
# 学习板上 D9  对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 构造接口 标准 MicroPython 的 machine.IIC 模块
#   I2C(id)
#   id      串口编号    |   必要参数 本固件支持 [0, 3] 总共 4 个 I2C 模块
#                       |   HW-I2C  | Logical | SCL | SDA |
#                       |   LPI2C1  | id = 0  | B30 | B31 |
#                       |   LPI2C2  | id = 1  | C19 | C18 |
#                       |   LPI2C3  | id = 2  | B8  | B9  |
#                       |   LPI2C4  | id = 3  | D22 | D23 |
#   freq    传输速率    |   可选参数 默认 400000
iic4 = I2C(3, freq = 100000)

# 其余接口：
# device_list = I2C.scan()                          # 扫描 IIC 总线上是否有设备 范围从 0x08 到 0x77 输出一个响应的地址列表
# rx_byte = I2C.readfrom(device_list[0], 1, True)   # 读取一个字节数据 True 发送停止信号 False 不发生停止信号
# I2C.readfrom_into(device_list[0], rx_buff, True)  # 读取 rx_buff 长度数据 True 发送停止信号 False 无停止信号
# I2C.writeto(device_list[0], tx_buff, True)        # 输出 tx_buff 长度数据 True 发送停止信号 False 无停止信号
# I2C.writevto(device_list[0], vectors, True)       # 输出 vectors 矩阵数据 True 发送停止信号 False 无停止信号

# 需要注意的是
# 任意的总线错误都会导致程序报错
# 任意的总线错误都会导致程序报错
# 任意的总线错误都会导致程序报错
# 包括 NACK 、 起始停止异常等

tx_buff = bytearray(b'1234')
rx_buff = bytearray(len(tx_buff))
vectors = [bytearray([0x12, 0x34]), bytearray([0x56, 0x78])]

# 扫描 IIC 总线上是否有设备 范围从 0x08 到 0x77 输出一个响应的地址列表
device_list = iic4.scan()
print(len(device_list), device_list)

while len(device_list):
    # 每 500ms 读取一次 将数据再原样发回
    time.sleep_ms(500)
    # 翻转 C4 LED 电平
    led.toggle()

    rx_byte = iic4.readfrom(device_list[0], 1, True)
    print(rx_byte)

    iic4.readfrom_into(device_list[0], rx_buff, True)
    print(rx_buff)

    iic4.writeto(device_list[0], tx_buff, True)
    iic4.writevto(device_list[0], vectors, True)
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
