
# 本示例程序演示如何使用 IPS200PRO 库
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的屏幕接口测试

# 本例程演示 IPS200PRO 屏幕的 时钟 组件使用
# 关联的通用接口也会一并演示使用方法与效果

# 例程效果是屏幕显示 时钟 并有一些变化

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

ips200pro.help_clock()

# 时钟 组件接口 :
# ------------------------------------------------------------------------------
#   新建一个 时钟 返回 时钟 索引号
#   clock_id = IPS200PRO.clock_create(x, y, size, type)
#       x               横向坐标    |   必要参数 组件 的 X 轴坐标
#       y               竖向坐标    |   必要参数 组件 的 Y 轴坐标
#       size            时钟尺寸    |   必要参数 模拟模式下是直径 [80, 240] 数字模式下是字体 IPS200PRO.FONT_SIZE_xxx
#                                       参数 xxx = (12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 40)
#       type            时钟类型    |   必要参数 IPS200PRO.(CLOCK_DIGITAL, CLOCK_ANALOG) 数字时钟 圆形时钟
#       return          返回内容    |   正常情况下返回对应 时钟 的索引
# ------------------------------------------------------------------------------
#   通过对象调用 输出模块的 clock 部分的使用帮助信息
#   IPS200PRO.help_clock()
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
#   修改组件字体
#   IPS200PRO.set_font(widgets_id, font)
#       widgets_id      组件索引    |   必要参数 组件 的索引号
#       font            字体大小    |   必要参数 IPS200PRO.FONT_SIZE_xxx
#                                       参数 xxx = (12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 40)
# ------------------------------------------------------------------------------
#   修改组件颜色
#   IPS200PRO.set_color(widgets_id, type, color)
#       widgets_id      组件索引    |   必要参数 组件 的索引号
#       type            设置选项    |   必要参数 指定要设置的是什么的颜色 IPS200PRO.COLOR_xxx
#                                       CLOCK 组件可用参数 xxx = (FOREGROUND, BACKGROUND, BORDER, CLOCK_HOUR, CLOCK_MINUTE, CLOCK_SECOND, CLOCK_TICKS)
#                                       对应 (前景色, 背景色, 组件边线颜色, 圆形时钟时针颜色, 圆形时钟分针颜色, 圆形时钟秒针颜色, 圆形时钟刻度颜色)
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
#   修改模组系统时间 会关联到 时钟 组件上
#   IPS200PRO.system_time(hour, minute, second)
#       hour            小时参数    |   必要参数 二十四小时制 输入 [0, 23]
#       minute          分钟参数    |   必要参数 六十分钟 输入 [0, 59]
#       second          秒钟参数    |   必要参数 六十秒钟 输入 [0, 59]
# ------------------------------------------------------------------------------

# 每个组件都是一个对象 对象之间可以设置继承关系 但只进行位置和范围的继承
# 设置父对象 就是将本对象 widgets_id1 设置与目标对象 widgets_id2 的继承关系
# 本对象就是子对象 目标对象就是父对象 此时子对象就只能在父对象的组件范围内操作
# 子对象的位置坐标就相对于父对象进行偏移 超出父对象大小范围的部分将不会显示
# 直接操作父对象的位置 子对象会随着父对象一同移动 可简单看做附着效果

# 新建 时钟 组件
clock_id = ips200pro.clock_create( 40,   0, 160, IPS200PRO.CLOCK_ANALOG)
time.sleep_ms(500)

# 重新设置 时钟 的字体 它会立即生效
ips200pro.set_font(clock_id, IPS200PRO.FONT_SIZE_12)
time.sleep_ms(500)

# 修改 时钟 的 前景色 背景色 边线颜色
ips200pro.set_color(clock_id, IPS200PRO.COLOR_FOREGROUND   , 0x0000)
ips200pro.set_color(clock_id, IPS200PRO.COLOR_BACKGROUND   , 0x0000)
ips200pro.set_color(clock_id, IPS200PRO.COLOR_BORDER       , 0x0000)
# 修改 圆形时钟时针颜色 圆形时钟分针颜色 圆形时钟秒针颜色 圆形时钟刻度颜色
# 数字时钟则没有以下四个属性设置
ips200pro.set_color(clock_id, IPS200PRO.COLOR_CLOCK_HOUR   , 0xFFFF)
ips200pro.set_color(clock_id, IPS200PRO.COLOR_CLOCK_MINUTE , 0xFFFF)
ips200pro.set_color(clock_id, IPS200PRO.COLOR_CLOCK_SECOND , 0xFFFF)
ips200pro.set_color(clock_id, IPS200PRO.COLOR_CLOCK_TICKS  , ips200pro.rgb888_to_rgb565(0x39C5BB))
time.sleep_ms(500)

# 重新设置 时钟 的位置
ips200pro.set_position(clock_id, 40, 65)
time.sleep_ms(500)

# 显示与隐藏 时钟
ips200pro.set_hidden(clock_id, True)
time.sleep_ms(500)
ips200pro.set_hidden(clock_id, False)
time.sleep_ms(500)

# 设置 时钟 的依附关系 将其切换到另一个 页面 下
ips200pro.set_parent(clock_id, page_id1)
time.sleep_ms(500)

# 切换页面 可以看到刚刚操作的 时钟 已经切换过来了
# 需要注意的是 开启动画后需要 1s 的动画时间 视情况来决定是否开启动画
ips200pro.page_switch(page_id1, IPS200PRO.PAGE_ANIM_ON)
time.sleep_ms(1000)

# 重新设置 时钟 的位置
ips200pro.set_position(clock_id, 40, 65)
time.sleep_ms(500)

# 重新设置 系统时间 它会同步到 时钟 组件并刷新显示 系统时间 会保存在屏幕中
ips200pro.system_time(10, 2, 4)

while True:
    time.sleep_ms(1000)
    
    led.toggle()
    gc.collect()
