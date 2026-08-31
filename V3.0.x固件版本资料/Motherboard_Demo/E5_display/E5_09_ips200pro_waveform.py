
# 本示例程序演示如何使用 IPS200PRO 库
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的屏幕接口测试

# 本例程演示 IPS200PRO 屏幕的 波形图 组件使用
# 关联的通用接口也会一并演示使用方法与效果

# 例程效果是屏幕显示 波形图 并有动态刷新波形

# 从 machine 库包含所有内容
from machine import *

# 从 seekfree 库包含所有内容
from seekfree import *

# 从 array 库包含所有内容
from array import *

# 包含 gc time math 类
import gc, time, math

# 延迟上电 避免 CR 时序控制导致屏幕还未成功启动
time.sleep_ms(100)

led = Pin('C4' , Pin.OUT, value = True)

IPS200PRO.help()
ips200pro = IPS200PRO(IPS200PRO.TITLE_BOTTOM, 30)
page_id1 = ips200pro.page_create("页面1")
page_id2 = ips200pro.page_create("页面2")
ips200pro.info()

ips200pro.help_waveform()

# 波形图 组件接口 :
# ------------------------------------------------------------------------------
#   新建一个 波形图 返回 波形图 索引号
#   waveform_id = IPS200PRO.waveform_create(x, y, width, height)
#       x               横向坐标    |   必要参数 波形图 的 X 轴坐标
#       y               竖向坐标    |   必要参数 波形图 的 Y 轴坐标
#       width           组件宽度    |   必要参数 波形图 的宽度
#       height          组件高度    |   必要参数 波形图 的高度
#       return          返回内容    |   正常情况下返回对应 波形图 的索引
# ------------------------------------------------------------------------------
#   通过对象调用 输出模块的 waveform 部分的使用帮助信息
#   IPS200PRO.help_waveform()
# ------------------------------------------------------------------------------
#   向 波形图 指定 ID 的波形添加数据 可以输入字节数组 或添加单个数据
#   IPS200PRO.waveform_value(waveform_id, line_id, data_list, color = 0xF800)
#       waveform_id     波形索引    |   必要参数 波形图 的索引号
#       line_id         线条索引    |   必要参数 线条 的索引号 一个波形图中最多五条线 范围为 [1, 5]
#       data_list       数据列表    |   必要参数 数据 传入一个整形数组 数值范围是 [0, 100]
#       color           线条颜色    |   可选参数 线条 颜色 RGB565 格式的颜色 默认红色
#   IPS200PRO.waveform_value(waveform_id, line_id, data, color = 0xF800)
#       waveform_id     波形索引    |   必要参数 波形图 的索引号
#       line_id         线条索引    |   必要参数 线条 的索引号 一个波形图中最多五条线 范围为 [1, 5]
#       data            数据数值    |   必要参数 数据 传入一个整形数值 数值范围是 [0, 100]
#       color           线条颜色    |   可选参数 线条 颜色 RGB565 格式的颜色 默认红色
# ------------------------------------------------------------------------------
#   修改指定 波形图 中 线条 的显示状态
#   IPS200PRO.waveform_line(waveform_id, line_id, enable = True)
#       waveform_id     波形索引    |   必要参数 波形图 的索引号
#       line_id         线条索引    |   必要参数 线条 的索引号 一个波形图中最多五条线 范围为 [1, 5]
#       enable          线条状态    |   可选参数 默认为 True 显示线条 可输入 False 隐藏显示
# ------------------------------------------------------------------------------
#   设置 波形图 中 线条 的显示模式
#   IPS200PRO.waveform_mode(waveform_id, connect)
#       waveform_id     波形索引    |   必要参数 波形图 的索引号
#       connect         线条模式    |   可选参数 True 点之间连接折线显示 False 点之间不连接散点显示
# ------------------------------------------------------------------------------
#   清空 波形图 中所有 线条 数据
#   IPS200PRO.waveform_clear(waveform_id)
#       waveform_id     波形索引    |   必要参数 波形图 的索引号
# ------------------------------------------------------------------------------

# 屏幕通用组件接口 :
# ------------------------------------------------------------------------------
#   RGB888 转换 RGB565 接口
#   rgb565_16bit = IPS200PRO.rgb888_to_rgb565(rgb888_24bit)
#       rgb888_24bit    颜色数值    |   必要参数 一个 RGB888 格式的 24bit 色彩值
#       return          返回内容    |   返回一个 RGB565 格式的 16bit 色彩值
#   rgb565_16bit = IPS200PRO.rgb888_to_rgb565(red_8bit, green_8bit, blue_8bit)
#       red_8bit        红色数值    |   必要参数 一个 RGB888 格式的 8bit R 色彩值分量
#       green_8bit      绿色数值    |   必要参数 一个 RGB888 格式的 8bit G 色彩值分量
#       blue_8bit       蓝色数值    |   必要参数 一个 RGB888 格式的 8bit B 色彩值分量
#       return          返回内容    |   返回一个 RGB565 格式的 16bit 色彩值
# ------------------------------------------------------------------------------
#   修改组件颜色
#   IPS200PRO.set_color(widgets_id, type, color)
#       widgets_id      组件索引    |   必要参数 组件 的索引号
#       type            设置选项    |   必要参数 指定要设置的是什么的颜色 IPS200PRO.COLOR_xxx
#                                       WAVEFORM 组件可用参数 xxx = (BACKGROUND)
#                                       对应 (背景色)
#       color           颜色数值    |   必要参数 RGB565 格式的颜色
# ------------------------------------------------------------------------------
#   修改组件位置
#   IPS200PRO.set_position(widgets_id, x, y)
#       widgets_id      组件索引    |   必要参数 组件 的索引号
#       x               横向坐标    |   必要参数 组件 的 X 轴坐标
#       y               竖向坐标    |   必要参数 组件 的 Y 轴坐标
# ------------------------------------------------------------------------------
#   设置组件隐藏显示
#   IPS200PRO.set_hidden(widgets_id, enable)
#       widgets_id      组件索引    |   必要参数 组件 的索引号
#       enable          隐藏使能    |   必要参数 True 隐藏 False 取消隐藏
# ------------------------------------------------------------------------------
#   修改组件父对象 可用于将组件切换到其它页面 或者给组件嵌套组件
#   IPS200PRO.set_parent(widgets_id1, widgets_id2)
#       widgets_id1     组件索引    |   必要参数 组件 的索引号 子对象
#       widgets_id2     组件索引    |   必要参数 组件 的索引号 父对象
# ------------------------------------------------------------------------------

# 每个组件都是一个对象 对象之间可以设置继承关系 但只进行位置和范围的继承
# 设置父对象 就是将本对象 widgets_id1 设置与目标对象 widgets_id2 的继承关系
# 本对象就是子对象 目标对象就是父对象 此时子对象就只能在父对象的组件范围内操作
# 子对象的位置坐标就相对于父对象进行偏移 超出父对象大小范围的部分将不会显示
# 直接操作父对象的位置 子对象会随着父对象一同移动 可简单看做附着效果

# 新建 波形图 组件
waveform_id1 = ips200pro.waveform_create(  0,   0, 240, 100)
waveform_id2 = ips200pro.waveform_create(  0, 100, 240, 100)
time.sleep_ms(500)

wave_sin_1 = array('h', [0] * (250))
wave_sin_2 = array('h', [0] * (200))
wave_sin_3 = array('h', [0] * (150))
wave_sin_4 = array('h', [0] * (100))
wave_sin_5 = array('h', [0] * ( 50))

# 生成正弦波形数组
#   generate_sine_wave(min_val, max_val, arr)
#   min_val         最大幅值    |   必要参数 波峰的数值
#   max_val         最小幅值    |   必要参数 波谷的数值
#   arr             数组对象    |   必要参数 数组对象用来存放波形
def generate_sine_wave (min_val, max_val, arr):
    length = len(arr)
    if min_val >= max_val:
        raise ValueError("最小值必须小于最大值")
    if length <= 0:
        raise ValueError("长度必须为正整数")
    
    # 计算振幅和偏移量
    amplitude = (max_val - min_val) / 2.0
    offset = min_val + amplitude
    
    # 生成正弦波数据
    for i in range(length):
        # 计算角度（0到2π之间的一个完整周期）
        angle = 2 * math.pi * i / length
        # 计算正弦值并缩放到指定范围
        value = amplitude * math.sin(angle) + offset
        arr[i] = int(value)

generate_sine_wave(  0, 100, wave_sin_1)
generate_sine_wave( 10,  90, wave_sin_2)
generate_sine_wave( 20,  80, wave_sin_3)
generate_sine_wave( 30,  70, wave_sin_4)
generate_sine_wave( 40,  60, wave_sin_5)

# 向 波形图 组件中添加数据 直接输入整个数组 和 单个数据输入
ips200pro.waveform_value(waveform_id1, 1, wave_sin_1, 0xF800)
ips200pro.waveform_value(waveform_id1, 2, wave_sin_2, 0x07E0)
ips200pro.waveform_value(waveform_id1, 3, wave_sin_3, 0x001F)
ips200pro.waveform_value(waveform_id1, 4, wave_sin_4, 0xFFFF)
ips200pro.waveform_value(waveform_id1, 5, wave_sin_5, 0x8430)
for i in range(240):
    ips200pro.waveform_value(waveform_id2, 1, (75) if ((i // 10) % 2) else (25), 0x0000)
time.sleep_ms(500)

# 修改 波形图 组件中线条显示状态
ips200pro.waveform_line(waveform_id1, 1, False)
ips200pro.waveform_line(waveform_id1, 2, False)
time.sleep_ms(500)
ips200pro.waveform_line(waveform_id1, 1, True)
ips200pro.waveform_line(waveform_id1, 2)
time.sleep_ms(500)

# 修改 波形图 组件中线条显示模式
ips200pro.waveform_mode(waveform_id2, True)
time.sleep_ms(500)
ips200pro.waveform_mode(waveform_id2, False)
time.sleep_ms(500)
ips200pro.waveform_mode(waveform_id2, True)
time.sleep_ms(500)

# 清空 波形图 中 线条 数据
ips200pro.waveform_clear(waveform_id1)
time.sleep_ms(500)

# 修改 波形图 的 背景色
ips200pro.set_color(waveform_id1, IPS200PRO.COLOR_BACKGROUND, 0x0000)
ips200pro.set_color(waveform_id2, IPS200PRO.COLOR_BACKGROUND, 0xF800)
time.sleep_ms(500)

# 重新设置 波形图 的位置
ips200pro.set_position(waveform_id1, 0, 100)
ips200pro.set_position(waveform_id2, 0,   0)
time.sleep_ms(500)

# 显示与隐藏 波形图
ips200pro.set_hidden(waveform_id2, True)
time.sleep_ms(500)
ips200pro.set_hidden(waveform_id2, False)
time.sleep_ms(500)

# 设置 波形图 的依附关系 将其切换到另一个 页面 下
ips200pro.set_parent(waveform_id2, page_id1)
time.sleep_ms(500)

# 切换页面 可以看到刚刚操作的 波形图 已经切换过来了
# 需要注意的是 开启动画后需要 1s 的动画时间 视情况来决定是否开启动画
ips200pro.page_switch(page_id1, IPS200PRO.PAGE_ANIM_ON)
time.sleep_ms(1000)

# 将 波形图 依次切换过来 设置位置
ips200pro.set_position(waveform_id2, 0, 100)
time.sleep_ms(500)
ips200pro.set_parent(waveform_id1, page_id1)
time.sleep_ms(500)
ips200pro.set_position(waveform_id1, 0, 0)

data_index = 0

while True:
    time.sleep_ms(50)
    
    ips200pro.waveform_value(waveform_id1, 1, wave_sin_1[int(data_index % 250)])
    ips200pro.waveform_value(waveform_id1, 2, wave_sin_2[int(data_index % 200)])
    ips200pro.waveform_value(waveform_id1, 3, wave_sin_3[int(data_index % 150)])
    ips200pro.waveform_value(waveform_id1, 4, wave_sin_4[int(data_index % 100)])
    ips200pro.waveform_value(waveform_id1, 5, wave_sin_5[int(data_index %  50)])
    ips200pro.waveform_value(waveform_id2, 1, (75) if ((data_index // 10) % 2) else (25))
    data_index = (0) if(2999 == data_index) else (data_index + 1)

    led.toggle()
    gc.collect()
