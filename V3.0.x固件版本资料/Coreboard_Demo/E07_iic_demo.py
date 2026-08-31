
# 本示例程序演示如何使用 machine 库的 IIC 类接口
# 使用 RT1021-MicroPython 核心板
# 可以接入传感器测试

# 示例程序运行效果为每 500ms(0.5s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 并通过对应引脚进行 IIC 数据传输
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc, time

# 延迟上电 避免 CR 时序控制导致屏幕还未成功启动
time.sleep_ms(100)

# 核心板上 C4 是 LED
# 学习板上 D9  对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# ------------------------------------------------------------------------------
#   构造接口 标准 MicroPython 的 machine.IIC 模块
#   I2C_obj = I2C(id, freq = 400000)
#       id              接口编号    |   必要参数 本固件支持 (0, 1, 3) 总共 3 个 I2C 模块
#       freq            传输速率    |   可选参数 默认 400000
#       return          返回内容    |   正常情况下返回对应 I2C 的对象
# ------------------------------------------------------------------------------
# 接口编号与对应 ID 和 引脚 对应表
# | HW-I2C | Logical | SCL | SDA |
# | LPI2C1 | id = 0  | B30 | B31 |
# | LPI2C2 | id = 1  | C19 | C18 |
# | LPI2C4 | id = 3  | D22 | D23 |
iic4 = I2C(3, freq = 100000)

# I2C 接口 :
# ------------------------------------------------------------------------------
#   扫描 IIC 总线上是否有设备
#   device_list = I2C.scan()
#       return          返回内容    |   返回从 0x08 到 0x77 地址有响应的从机地址列表 字节数组形式
# ------------------------------------------------------------------------------
#   从指定地址的从机读取指定长度的数据
#   rx_buff = I2C.readfrom(addr, lenght, stop = True)
#       addr            目标地址    |   必要参数 目标通信器件的地址
#       lenght          读取长度    |   必要参数 准备读取的数据长度
#       stop            停止条件    |   可选参数 True 发送停止信号 False 不发生停止信号
#       return          返回内容    |   返回读取到的内容 为字节数组形式
# ------------------------------------------------------------------------------
#   从指定地址的从机读取缓冲区长度的数据
#   I2C.readfrom_into(addr, rx_buff, stop = True)
#       addr            目标地址    |   必要参数 目标通信器件的地址
#       rx_buff         数据缓冲    |   必要参数 存放读取数据的缓冲区
#       stop            停止条件    |   可选参数 True 发送停止信号 False 不发生停止信号
# ------------------------------------------------------------------------------
#   向指定地址的从机输出数据缓冲区长度数据
#   I2C.writeto(addr, tx_buff, stop = True)
#       addr            目标地址    |   必要参数 目标通信器件的地址
#       tx_buff         数据缓冲    |   必要参数 存放发送数据的缓冲区
#       stop            停止条件    |   可选参数 True 发送停止信号 False 不发生停止信号
# ------------------------------------------------------------------------------
#   向指定地址的从机输出矩阵数据
#   I2C.writevto(addr, vectors, stop = True)
#       addr            目标地址    |   必要参数 目标通信器件的地址
#       vectors         矩阵缓冲    |   必要参数 存放发送数据的矩阵缓冲区
#       stop            停止条件    |   可选参数 True 发送停止信号 False 不发生停止信号
# ------------------------------------------------------------------------------

# 任意的总线错误都会导致程序报错 包括 NACK 、 起始停止异常等
# 任意的总线错误都会导致程序报错 包括 NACK 、 起始停止异常等
# 任意的总线错误都会导致程序报错 包括 NACK 、 起始停止异常等

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
