
# 本示例程序演示如何使用 machine 库的 SPI 类接口
# 使用 RT1021-MicroPython 核心板
# 可以用杜邦线将 MOSI 与 MISO 短接测试 也可以接入传感器

# 示例程序运行效果为每 500ms(0.5s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 并通过对应引脚进行 SPI 数据传输
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

# 构造接口 标准 MicroPython 的 machine.SPI 模块
#   SPI(id)
#   id      串口编号    |   必要参数 本固件支持 [0, 3] 总共 4 个 SPI 模块
#                       |   HW-UART   | Logical | SCK | MOSI | MISO | CS0 |
#                       |   LPSPI1    | id = 0  | B10 | B12  | B13  | B11 |
#                       |   LPSPI2    | id = 1  | C10 | C12  | C13  | C11 |
#                       |   LPSPI3    | id = 2  | B28 | B30  | B31  | B29 |
#                       |   LPSPI4    | id = 3  | B18 | B20  | B21  | B19 |
spi2 = SPI(1)

# SPI 参数设置 参数说明
#   SPI.init(baudrate = 1000000, polarity = 0, phase = 0)
#   baudrate    传输速率    | 默认 1000000 1Mbps
#   polarity    电平极性    | 可选参数 默认 0 {0 - 时钟空闲时低电平, 1 - 时钟空闲时高电平}
#   phase       时钟相位    | 可选参数 默认 0 {0 - 第一个时钟沿采样数据, 1 - 第二个时钟沿采样数据}
spi2.init(baudrate = 1000000, polarity = 0, phase = 0)

# 其余接口：
# rx_byte = spi.read(1)                 # 读取一个字节数据 默认输出 0x00
# spi.readinto(rx_buff)                 # 读取 rx_buff 长度数据 默认输出 0x00
# spi.write(tx_buff)                    # 输出 tx_buff 长度数据
# spi.write_readinto(tx_buff, rx_buff)  # 输出 tx_buff 长度数据 同时读取数据到 rx_buff 这两个缓冲区必须一样长

tx_buff = bytearray(b'1234')
rx_buff = bytearray(len(tx_buff))

while True:
    # 每 500ms 读取一次 将数据再原样发回
    time.sleep_ms(500)
    # 翻转 C4 LED 电平
    led.toggle()

    rx_byte = spi2.read(1)
    spi2.readinto(rx_buff)
    spi2.write(tx_buff)
    spi2.write_readinto(tx_buff, rx_buff)

    # 把 MOSI 和 MISO 接到一起可以看到输出输入是一样的数据
    print("write_readinto out:", tx_buff)
    print("write_readinto in :", rx_buff)
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
