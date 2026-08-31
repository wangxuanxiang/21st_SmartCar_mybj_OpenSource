
# 本示例程序演示如何使用 machine 库的 UART 类接口
# 使用 RT1021-MicroPython 核心板
# 搭配 USB 转 TTL 模块进行 UART 通信测试

# 示例程序运行效果为每 500ms(0.5s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 并通过 UART 接收并回传数据
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

# 构造接口 标准 MicroPython 的 machine.UART 模块
#   UART(id)
#   id      串口编号    |   必要参数 本固件支持 [0, 7] 总共 8 个 UART 模块
#                       |   HW-UART | Logical | TX   | RX  |
#                       |   LPUART1 | id = 0  | B6   | B7  |
#                       |   LPUART2 | id = 1  | C22  | C23 |
#                       |   LPUART3 | id = 2  | C6   | C7  |
#                       |   LPUART4 | id = 3  | D0   | D1  |
#                       |   LPUART5 | id = 4  | B10  | B11 |
#                       |   LPUART6 | id = 5  | D20  | D21 |
#                       |   LPUART7 | id = 6  | D17  | D18 |
#                       |   LPUART8 | id = 7  | D22  | D23 |
uart6 = UART(5)
uart8 = UART(7)

# 串口参数设置
#   UART.init(baudrate=9600, bits=8, parity=None, stop=1, *, ...)
#   baudrate    串口速率    | 必要参数 默认 9600
#   bits        数据位数    | 可选参数 默认 8 bits 数据位
#   parity      校验位数    | 可选参数 默认 None {None, 0, 1} 无校验, 偶校验, 奇校验
#   stop        停止位数    | 可选参数 默认 1 bit 停止位
uart6.init(460800)
uart8.init(460800)

# 其余接口：
# buf = UART.read(n)    # 读取 n字节到 buf
# UART.readinto(buf)    # 读取数据节到 buf
# UART.write(buf)       # 将 buf 内容通过 UART 发送
# UART.any()            # 判断 UART 是否有数据可读取

uart6.write("Test.\r\n")
uart8.write("Test.\r\n")
buf_len = 0

while True:
    # 每 500ms 读取一次 将数据再原样发回
    time.sleep_ms(500)
    # 翻转 C4 LED 电平
    led .toggle()
    
    buf_len = uart6.any()
    if(buf_len):
        buf = uart6.read(buf_len)
        print("uart6 buf_len ={:>6d}".format(buf_len))
        uart6.write("uart6:")
        uart6.write(buf)

    buf_len = uart8.any()
    if(buf_len):
        buf = uart8.read(buf_len)
        print("uart8 buf_len ={:>6d}".format(buf_len))
        uart8.write("uart8:")
        uart8.write(buf)
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
