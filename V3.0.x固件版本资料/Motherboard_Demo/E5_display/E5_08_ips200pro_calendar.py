
# 本示例程序演示如何使用 IPS200PRO 库
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的屏幕接口测试

# 本例程演示 IPS200PRO 屏幕的 日历 组件使用
# 关联的通用接口也会一并演示使用方法与效果

# 例程效果是屏幕显示 日历 并有一些变化
# 最终实现一个切换效果

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

ips200pro.help_calendar()

# 日历 组件接口 :
# ------------------------------------------------------------------------------
#   新建一个 日历 返回 日历 索引号
#   calendar_id = IPS200PRO.calendar_create(x, y, width, height)
#       x               横向坐标    |   必要参数 日历 的 X 轴坐标
#       y               竖向坐标    |   必要参数 日历 的 Y 轴坐标
#       width           日历宽度    |   必要参数 日历 的宽度
#       height          日历高度    |   必要参数 日历 的高度
#       return          返回内容    |   正常情况下返回对应 日历 的索引
# ------------------------------------------------------------------------------
#   通过对象调用 输出模块的 calendar 部分的使用帮助信息
#   IPS200PRO.help_calendar()
# ------------------------------------------------------------------------------
#   日历 定位到指定年月显示
#   IPS200PRO.calendar_locate(year, month, mode)
#       year            定位年份    |   必要参数 年份范围是 [1970, 2099]
#       month           定位月份    |   必要参数 月份范围是 [1, 12]
#       mode            显示模式    |   必要参数 IPS200PRO.(CALENDAR_CHINESE, CALENDAR_ENGLISH) 中英文模式
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
#                                       需要注意 仅 16、20、24 号字体支持中文显示
# ------------------------------------------------------------------------------
#   修改组件颜色
#   IPS200PRO.set_color(widgets_id, type, color)
#       widgets_id      组件索引    |   必要参数 组件 的索引号
#       type            设置选项    |   必要参数 指定要设置的是什么的颜色 IPS200PRO.COLOR_xxx
#                                       CALENDAR 组件可用参数 xxx = (FOREGROUND, BACKGROUND, BORDER, CALENDAR_YEAR, CALENDAR_WEEK, CALENDAR_TODAY)
#                                       对应 (前景色, 背景色, 组件边线颜色, 年份字体颜色, 月份字体颜色, 今日日期框选颜色)
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
#   修改模组系统日期 会关联到 日历 组件上
#   IPS200PRO.system_date(year, month, day)
#       year            年份参数    |   必要参数 年份范围是 [1970, 2099]
#       month           月份参数    |   必要参数 月份范围是 [1, 12]
#       day             日期参数    |   必要参数 日期范围是 1 到 当月最大天数
# ------------------------------------------------------------------------------

# 每个组件都是一个对象 对象之间可以设置继承关系 但只进行位置和范围的继承
# 设置父对象 就是将本对象 widgets_id1 设置与目标对象 widgets_id2 的继承关系
# 本对象就是子对象 目标对象就是父对象 此时子对象就只能在父对象的组件范围内操作
# 子对象的位置坐标就相对于父对象进行偏移 超出父对象大小范围的部分将不会显示
# 直接操作父对象的位置 子对象会随着父对象一同移动 可简单看做附着效果

# 新建 日历 组件
calendar_id = ips200pro.calendar_create(  0,   0, 240, 240)
time.sleep_ms(500)

# 日历 定位到指定年月显示
ips200pro.calendar_locate(1970, 1, IPS200PRO.CALENDAR_CHINESE)
time.sleep_ms(500)

# 重新设置 日历 的字体 它会立即生效
ips200pro.set_font(calendar_id, IPS200PRO.FONT_SIZE_20)
time.sleep_ms(500)

# 修改 日历 的 前景色 背景色 边线颜色 年份字体颜色 月份字体颜色 今日日期框选颜色
ips200pro.set_color(calendar_id, IPS200PRO.COLOR_FOREGROUND     , ips200pro.rgb888_to_rgb565(0x3B54D0))
ips200pro.set_color(calendar_id, IPS200PRO.COLOR_BACKGROUND     , ips200pro.rgb888_to_rgb565(0xDEF4FD))
ips200pro.set_color(calendar_id, IPS200PRO.COLOR_BORDER         , ips200pro.rgb888_to_rgb565(0x408EAF))
ips200pro.set_color(calendar_id, IPS200PRO.COLOR_CALENDAR_YEAR  , ips200pro.rgb888_to_rgb565(0x39C5BB))
ips200pro.set_color(calendar_id, IPS200PRO.COLOR_CALENDAR_WEEK  , ips200pro.rgb888_to_rgb565(0x39C5BB))
ips200pro.set_color(calendar_id, IPS200PRO.COLOR_CALENDAR_TODAY , ips200pro.rgb888_to_rgb565(0x66CCFF))
time.sleep_ms(500)

# 重新设置 日历 的位置
ips200pro.set_position(calendar_id, 0, 60)
time.sleep_ms(500)

# 显示与隐藏 日历
ips200pro.set_hidden(calendar_id, True)
time.sleep_ms(500)
ips200pro.set_hidden(calendar_id, False)
time.sleep_ms(500)

# 设置 日历 的依附关系 将其切换到另一个 页面 下
ips200pro.set_parent(calendar_id, page_id1)
time.sleep_ms(500)

# 切换页面 可以看到刚刚操作的 日历 已经切换过来了
# 需要注意的是 开启动画后需要 1s 的动画时间 视情况来决定是否开启动画
ips200pro.page_switch(page_id1, IPS200PRO.PAGE_ANIM_ON)
time.sleep_ms(1000)

# 重新设置 系统日期 它会同步到 日历 组件并刷新显示 系统日期 会保存在屏幕中
ips200pro.system_date(2025, 8, 20)  # 20 届国赛日期

dis_year  = 2025
dis_month = 8

while True:
    time.sleep_ms(1000)
    
    ips200pro.calendar_locate(dis_year, dis_month, (IPS200PRO.CALENDAR_CHINESE)if(dis_month%2)else(IPS200PRO.CALENDAR_ENGLISH))
    
    dis_year   = (dis_year + 1) if(1 == dis_month) else ((1970) if(12 == dis_month and 2099 == dis_year) else (dis_year))
    dis_month  = (1) if(12 == dis_month) else (dis_month + 1)
    
    led.toggle()
    gc.collect()
