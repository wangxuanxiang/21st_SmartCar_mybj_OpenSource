
# 本示例程序演示如何使用 IPS200PRO 库
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的屏幕接口测试

# 本例程演示 IPS200PRO 屏幕的 容器 组件使用
# 关联的通用接口也会一并演示使用方法与效果

# 例程效果是屏幕显示 容器 并附着一些其他组件
# 最终实现一个 容器 搭配的效果

# 从 machine 库包含所有内容
from machine import *

# 从 seekfree 库包含所有内容
from seekfree import *

# 包含 gc 与 time 类
import gc, time

# 延迟上电 避免 CR 时序控制导致屏幕还未成功启动
time.sleep_ms(100)

led = Pin('C4' , Pin.OUT, value = True)

IPS200PRO.help()
ips200pro = IPS200PRO(IPS200PRO.TITLE_BOTTOM, 30)
page_id1 = ips200pro.page_create("页面1")
page_id2 = ips200pro.page_create("页面2")
ips200pro.info()

ips200pro.help_container()

# 容器 组件接口 :
# ------------------------------------------------------------------------------
#   新建一个 容器 返回 容器 索引号
#   container_id = IPS200PRO.container_create(x, y, width, height)
#       x               横向坐标    |   必要参数 容器 的 X 轴坐标
#       y               竖向坐标    |   必要参数 容器 的 Y 轴坐标
#       width           容器宽度    |   必要参数 容器 的宽度
#       height          容器高度    |   必要参数 容器 的高度
#       return          返回内容    |   正常情况下返回对应 容器 的索引
# ------------------------------------------------------------------------------
#   通过对象调用 输出模块的 container 部分的使用帮助信息
#   IPS200PRO.help_container()
# ------------------------------------------------------------------------------
#   修改 容器 样式
#   IPS200PRO.container_radius(container_id, border_width, radius)
#       container_id    容器索引    |   必要参数 容器 的索引号
#       border_width    边线宽度    |   必要参数 容器 边线宽度
#       radius          圆角半径    |   必要参数 容器 圆角半径
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
#                                       CONTAINER 组件可用参数 xxx = (BACKGROUND, BORDER)
#                                       对应 (背景色, 组件边线颜色)
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

# 新建 容器 组件
container_id1 = ips200pro.container_create(  0,   0, 240, 240)
container_id2 = ips200pro.container_create(  0,   0, 120, 120)
container_id3 = ips200pro.container_create(120,   0, 120, 120)
time.sleep_ms(500)

# 修改 容器 样式
ips200pro.container_radius(container_id1, 0,  0)
ips200pro.container_radius(container_id2, 2, 20)
ips200pro.container_radius(container_id3, 2, 20)
time.sleep_ms(500)

# 修改 容器 的 背景色 边线颜色
ips200pro.set_color(container_id1, IPS200PRO.COLOR_BACKGROUND, ips200pro.rgb888_to_rgb565(0xACACAC))
ips200pro.set_color(container_id1, IPS200PRO.COLOR_BORDER    , ips200pro.rgb888_to_rgb565(0x000000))
ips200pro.set_color(container_id2, IPS200PRO.COLOR_BACKGROUND, ips200pro.rgb888_to_rgb565(0x66CCFF))
ips200pro.set_color(container_id2, IPS200PRO.COLOR_BORDER    , ips200pro.rgb888_to_rgb565(0xDEDEDE))
ips200pro.set_color(container_id3, IPS200PRO.COLOR_BACKGROUND, ips200pro.rgb888_to_rgb565(0x39C5BB))
ips200pro.set_color(container_id3, IPS200PRO.COLOR_BORDER    , ips200pro.rgb888_to_rgb565(0xDEDEDE))
time.sleep_ms(500)

# 重新设置 容器 的位置
ips200pro.set_position(container_id3, 120, 120)
time.sleep_ms(500)

# 显示与隐藏 容器
ips200pro.set_hidden(container_id1, True)
time.sleep_ms(500)
ips200pro.set_hidden(container_id2, True)
time.sleep_ms(500)
ips200pro.set_hidden(container_id3, True)
time.sleep_ms(500)
ips200pro.set_hidden(container_id1, False)
time.sleep_ms(500)
ips200pro.set_hidden(container_id2, False)
time.sleep_ms(500)
ips200pro.set_hidden(container_id3, False)
time.sleep_ms(500)

# 新建一些组件用于容器依附测试
label_id        = ips200pro.label_create        (  0,   0, 100,  20, "标签容器依附")
meter_id        = ips200pro.meter_create        (  0,   0, 118, IPS200PRO.METER_SPEED)
clock_id        = ips200pro.clock_create        (  0,   0, 116, IPS200PRO.CLOCK_ANALOG)
progress_bar_id   = ips200pro.progress_bar_create   (  0,   0, 116,  30)
waveform_id     = ips200pro.waveform_create     (  0,   0, 120, 120)
ips200pro.waveform_mode(waveform_id , True)
for i in range(120):
    ips200pro.waveform_value(waveform_id, 1, (90) if ((i // 10) % 2) else (30))
time.sleep_ms(500)

# 设置依附关系
# 将 仪表 和 波形图 附着在 容器 1 上
ips200pro.set_parent(meter_id, container_id1)
ips200pro.set_position(meter_id, 2, 120)
time.sleep_ms(500)
ips200pro.set_parent(waveform_id, container_id1)
ips200pro.set_position(waveform_id, 120, 0)
time.sleep_ms(500)

# 将 标签 附着在 进度条 上
ips200pro.set_parent(label_id, progress_bar_id)
ips200pro.set_position(label_id, 10, 6)
time.sleep_ms(500)

# 将 进度条 附着在 容器 2 上
ips200pro.set_parent(progress_bar_id, container_id2)
ips200pro.set_position(progress_bar_id, 2, 45)
time.sleep_ms(500)

# 将 时钟 附着在 容器 3 上
ips200pro.set_parent(clock_id, container_id3)
ips200pro.set_position(clock_id, 2, 2)
time.sleep_ms(500)

# 将 容器 2/3 附着在大 容器 1 上
ips200pro.set_parent(container_id2, container_id1)
ips200pro.set_parent(container_id3, container_id1)
time.sleep_ms(500)
ips200pro.set_position(container_id3, 120, 120)
time.sleep_ms(500)

# 重新设置各组件颜色
ips200pro.set_color(label_id        , IPS200PRO.COLOR_FOREGROUND       , ips200pro.rgb888_to_rgb565(0x39C5BB))
ips200pro.set_color(progress_bar_id   , IPS200PRO.COLOR_FOREGROUND       , ips200pro.rgb888_to_rgb565(0xD80000))
ips200pro.set_color(progress_bar_id   , IPS200PRO.COLOR_BACKGROUND       , ips200pro.rgb888_to_rgb565(0x39C5BB))
ips200pro.set_color(waveform_id     , IPS200PRO.COLOR_BACKGROUND       , ips200pro.rgb888_to_rgb565(0x575757))
time.sleep_ms(500)

# 设置 容器 的依附关系 将其切换到另一个 页面 下
# 由于其他组件都依附在第一个 容器 所以切换第一个容器会将所有组件切换过去
ips200pro.set_parent(container_id1, page_id1)
time.sleep_ms(500)

# 切换页面 可以看到刚刚操作的 表格 已经切换过来了
# 需要注意的是 开启动画后需要 1s 的动画时间 视情况来决定是否开启动画
ips200pro.page_switch(page_id1, IPS200PRO.PAGE_ANIM_ON)
time.sleep_ms(1000)

progress_bar_step_dir = 2
progress_bar_step_value = 0

meter_step_dir = 1
meter_step_value = 0

waveform_value = 0

while True:
    time.sleep_ms(50)

    progress_bar_step_value = progress_bar_step_value + progress_bar_step_dir
    if progress_bar_step_value == 70:   progress_bar_step_dir = -2
    if progress_bar_step_value ==  0:   progress_bar_step_dir =  2
    ips200pro.progress_bar_value(progress_bar_id,  progress_bar_step_value,  progress_bar_step_value + 30)

    meter_step_value = meter_step_value + meter_step_dir
    if meter_step_value == 100:   meter_step_dir = -1
    if meter_step_value ==   0:   meter_step_dir =  1
    ips200pro.meter_value(meter_id, meter_step_value)

    ips200pro.waveform_value(waveform_id, 1, (90) if ((waveform_value // 10) % 2) else (30))
    waveform_value = (0) if(99 == waveform_value) else (waveform_value + 1)
    
    led.toggle()
    gc.collect()
