# 本示例程序演示如何使用 seekfree 库的 WIFI_SPI 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板与 WIFI_SPI 模块测试
# 当 D9 引脚电平出现变化时退出测试程序

# 示例程序运行效果是通过 WIFI_SPI 模块接收逐飞助手下发的调参数据
# 显示在 Thonny Shell 控制台并发送回逐飞助手的虚拟示波器显示
# 如果看到 Thonny Shell 控制台输出 Module init fault 报错
# 就证明 WIFI_SPI 模块连接异常 或者 热点名称 密码 不正确 无法正常连接网络
# 如果看到 Thonny Shell 控制台输出 Socket connect fault 报错
# 就证明 目标连接的 IP 地址或者端口不正确 无法建立网络通信连接
# 无法解决时请联系技术支持

# 默认使用虚拟示波器的快速数据发送接口节约发送时间

# 需要建立网络后打开 逐飞助手-虚拟示波器 界面！！！
# 需要建立网络后打开 逐飞助手-虚拟示波器 界面！！！
# 需要建立网络后打开 逐飞助手-虚拟示波器 界面！！！

# 如果想同时在串口助手串口能看到数据
# 可以选择使用虚拟示波器的 printf 协议
# 在 逐飞助手-虚拟示波器 界面的右下角有一个 printf 的开关
# 打开它后 使用 WIFI_SPI.send_str 发送数据

# 从 machine 库包含所有内容
from machine import *

# 从 seekfree 库包含 WIFI_SPI
from seekfree import WIFI_SPI

# 包含 gc time 类
import gc
import time

# 核心板上 C4 是 LED
# 学习板上 D9 对应二号拨码开关
led = Pin("C4", Pin.OUT, value=True)
switch2 = Pin("D9", Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# 显示帮助信息
WIFI_SPI.help()
time.sleep_ms(500)

# WIFI_SPI 初始化需要连接热点并与目标建立连接 因此需要几分钟的时间 请耐心等待
# WIFI_SPI 初始化需要连接热点并与目标建立连接 因此需要几分钟的时间 请耐心等待
# WIFI_SPI 初始化需要连接热点并与目标建立连接 因此需要几分钟的时间 请耐心等待

# 禁止在 WIFI_SPI 初始化过程中连续点击 Thonny 的 Stop 按钮 因为会导致底层异常中断抛出错误停止运行
# 禁止在 WIFI_SPI 初始化过程中连续点击 Thonny 的 Stop 按钮 因为会导致底层异常中断抛出错误停止运行
# 禁止在 WIFI_SPI 初始化过程中连续点击 Thonny 的 Stop 按钮 因为会导致底层异常中断抛出错误停止运行

# 构造接口 用于构建一个 WIFI_SPI 对象
#   WIFI_SPI(wifi_ssid, pass_word, connect_type, ip_addr, connect_port) # 构造接口 学习板的WIFI_SPI模块接口
#   wifi_ssid       热点名称    |   必要参数 WiFi 热点名称 字符串
#   pass_word       热点密码    |   必要参数 WiFi 热点密码 字符串
#   connect_type    连接类型    |   必要参数 连接类型 WIFI_SPI.TCP_CONNECT / WIFI_SPI.UDP_CONNECT
#   ip_addr         连接地址    |   必要参数 目标连接地址 字符串
#   connect_port    连接端口    |   必要参数 目标连接端口 字符串
wifi = WIFI_SPI(
    "WIFI_NAME", "WIFI_PASSWORD", WIFI_SPI.TCP_CONNECT, "192.168.1.13", "8086"
)

# 其余接口：
# WIFI_SPI.send_str(str)            # 发送字符串
#   str     字符数据    |   必要参数
# WIFI_SPI.send_oscilloscope(d1,[d2, d3, d4, d5, d6, d7, d8])  # 逐飞助手虚拟示波器数据上传
#   dx      波形数据    |   至少一个数据 最多可以填八个数据 数据类型支持浮点数
# WIFI_SPI.send_ccd_image(index)    # 逐飞助手 CCD 显示数据上传
#   index   接口编号    |   参数 [  CCD1_BUFFER_INDEX,      CCD2_BUFFER_INDEX,
#                       |           CCD3_BUFFER_INDEX,      CCD4_BUFFER_INDEX,
#                       |           CCD1_2_BUFFER_INDEX,    CCD3_4_BUFFER_INDEX]
#                       |   分别代表 仅显示 CCD1 图像 、 仅显示 CCD2 图像 、 选择两个 CCD 图像一起显示
# WIFI_SPI.data_analysis()          # 逐飞助手调参数据解析 会返回八个标志位的列表 标识各通道是否有数据更新
# WIFI_SPI.get_data()               # 逐飞助手调参数据获取 会返回八个数据的列表

wifi.info()
time.sleep_ms(500)

# data_analysis 数据解析接口 适配逐飞助手的无线调参功能
data_flag = wifi.data_analysis()
data_wave = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
for i in range(0, 8):
    # get_data 获取调参通道数据 只有一个参数范围 [0-7]
    data_wave[i] = wifi.get_data(i)

while True:
    time.sleep_ms(50)
    led.toggle()

    # 定期进行数据解析
    data_flag = wifi.data_analysis()
    for i in range(0, 8):
        # 判断哪个通道有数据更新
        if data_flag[i]:
            # 数据更新到缓冲
            data_wave[i] = wifi.get_data(i)
            # 将更新的通道数据输出到 Thonny 的控制台
            print("Data[{:<6}] updata : {:<.3f}.\r\n".format(i, data_wave[i]))

    # send_oscilloscope 将最多八个通道虚拟示波器数据上传到逐飞助手
    # 不需要这么多数据的话就只填自己需要的 只有两个数据就只填两个参数
    wifi.send_oscilloscope(
        data_wave[0],
        data_wave[1],
        data_wave[2],
        data_wave[3],
        data_wave[4],
        data_wave[5],
        data_wave[6],
        data_wave[7],
    )

    # 如果想同时在串口助手串口能看到数据
    # 可以选择使用虚拟示波器的 printf 协议
    # 在 逐飞助手-虚拟示波器 界面的右下角有一个 printf 的开关
    # 打开它后 使用 WIFI_SPI.send_str 发送数据
    # wifi.send_str("Data:{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(
    #     data_wave[0],data_wave[1],data_wave[2],data_wave[3],
    #     data_wave[4],data_wave[5],data_wave[6],data_wave[7]))

    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break

    # 回收内存
    gc.collect()
