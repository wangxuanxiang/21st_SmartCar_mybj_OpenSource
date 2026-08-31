
# 本示例程序演示如何使用 IPS200PRO 库
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的屏幕接口测试

# 本例程演示 IPS200PRO 屏幕的 页面 组件使用
# 因为页面是屏幕的基本组件 后续的功能都需要以页面为基础进行演示
# 关联的通用接口也会一并演示使用方法与效果

# 例程效果是屏幕显示页面 并有一些变化
# 最终在三个页面来回切换 伴随核心板 LED  C4 闪烁

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

# ------------------------------------------------------------------------------
#   构造接口 用于构建一个 IPS200PRO 对象
#   ips200pro_obj = IPS200PRO(title_position = IPS200PRO.TITLE_BOTTOM, title_high = 30)
#       title_position  标题位置    |   必要参数 IPS200PRO.TITLE_xxx
#                                       参数 xxx = (LEFT, RIGHT, TOP, BOTTOM)
#                                       对应 (左侧, 右侧, 上方, 下方) 默认为下方
#       title_high      标题高度    |   必要参数 标题栏的高度值 数值范围在 [1, 200]
#       return          返回内容    |   正常情况下返回对应 IPS200PRO 的对象
# ------------------------------------------------------------------------------
ips200pro = IPS200PRO(IPS200PRO.TITLE_BOTTOM, 30)

ips200pro.info()

ips200pro.help_page()

# 页面 组件接口 :
# ------------------------------------------------------------------------------
#   新建一个 页面 返回 页面 索引号
#   page_id = IPS200PRO.page_create(page_name)
#       page_name       页面名称    |   必要参数 支持中文英文 UTF-8 格式
#       return          返回内容    |   正常情况下返回对应 页面 的索引
# ------------------------------------------------------------------------------
#   通过对象调用 输出模块的 page 部分的使用帮助信息
#   IPS200PRO.help_page()
# ------------------------------------------------------------------------------
#   修改 页面 名称
#   IPS200PRO.page_name(page_id, page_name)
#       page_id         页面索引    |   必要参数 页面的索引号
#       page_name       页面名称    |   必要参数 支持中文英文 UTF-8 格式
# ------------------------------------------------------------------------------
#   切换到指定 页面
#   IPS200PRO.page_switch(page_id, anim_enable = IPS200PRO.PAGE_ANIM_OFF)
#       page_id         页面索引    |   必要参数 页面的索引号
#       anim_enable     动画使能    |   可选参数 IPS200PRO.(PAGE_ANIM_OFF, PAGE_ANIM_ON) 默认为 PAGE_ANIM_OFF 关闭动画
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
#       Tips : 驱动默认的字体大小是 16 可以在 ips200pro.info() 查看
#       Tips : 由于页面组件的特殊性 修改任意页面标题字体会对所有页面标题字体生效
#   IPS200PRO.set_font(widgets_id, font)
#       widgets_id      组件索引    |   必要参数 组件的索引号 索引为 0 时修改默认字体大小
#       font            字体大小    |   必要参数 IPS200PRO.FONT_SIZE_xxx
#                                       参数 xxx = (12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 40)
#                                       需要注意 仅 16、20、24 号字体支持中文显示
# ------------------------------------------------------------------------------
#   修改组件颜色
#       Tips : 由于页面组件的特殊性 修改任意页面标题颜色会对所有页面标题颜色生效
#   IPS200PRO.set_color(widgets_id, type, color)
#       widgets_id      组件索引    |   必要参数 组件 的索引号
#       type            设置选项    |   必要参数 指定要设置的是什么的颜色 IPS200PRO.COLOR_xxx
#                                       PAGE 组件可用参数 xxx = (FOREGROUND, BACKGROUND, PAGE_SELECTED_TEXT, PAGE_SELECTED_BG)
#                                       对应 (前景色, 背景色, 选中页面后的标题文字颜色, 选中页面后的标题背景颜色)
#       color           颜色数值    |   必要参数 RGB565 格式的颜色
# ------------------------------------------------------------------------------
#   设置组件隐藏显示
#   IPS200PRO.set_hidden(widgets_id, enable)
#       widgets_id      组件索引    |   必要参数 组件 的索引号
#       enable          隐藏使能    |   必要参数 True 隐藏 False 取消隐藏
# ------------------------------------------------------------------------------
#   修改背光亮度
#   IPS200PRO.set_backlight(backlight)
#       backlight       背光亮度    |   必要参数 背光亮度值 数值范围在 [1, 255]
# ------------------------------------------------------------------------------
#   修改屏幕显示方向
#   IPS200PRO.set_dir(dir)
#       dir             显示方向    |   必要参数 屏幕显示方向 IPS200PRO.xxx
#                                       可用参数 xxx = (PORTRAIT, PORTRAIT_180, CROSSWISE, CROSSWISE_180)
#                                       对应 (竖屏, 反转竖屏, 横屏, 反转横屏)
# ------------------------------------------------------------------------------

# 构建两 页面 构建成功会自己动跳转到新的页面
page_id1 = ips200pro.page_create("页面1")
time.sleep_ms(500)
page_id2 = ips200pro.page_create("page2")
time.sleep_ms(500)

# 更改第二个页面的名称
ips200pro.page_name(page_id2, "页面2")
time.sleep_ms(500)

# 通过 ID 切换回第一个页面
ips200pro.page_switch(page_id1)
time.sleep_ms(500)

# 第一个参数为 0 时修改默认字体 默认字体修改后后续的字体会变成默认字体
# 因此需要新建一个新的页面才能看到新字体生效
# 因为页面只有标题是文字 所以修改的是页面标题字体
# 由于页面组件的特殊性 修改任意页面标题字体会对所有页面标题字体生效
ips200pro.set_font(0, IPS200PRO.FONT_SIZE_24)
page_id3 = ips200pro.page_create("页面3")
time.sleep_ms(500)

# 传入 页面 ID 则修改页面的字体
ips200pro.set_font(page_id3, IPS200PRO.FONT_SIZE_16)
time.sleep_ms(500)

# 设置 页面 前景色 背景色 前景色就是标题字体颜色 背景色就是页面背景色
# 由于 页面 组件的特殊性 修改任意 页面 标题颜色会对所有 页面 标题颜色生效
ips200pro.set_color(page_id1, IPS200PRO.COLOR_FOREGROUND, ips200pro.rgb888_to_rgb565(0x39C5BB))
ips200pro.set_color(page_id1, IPS200PRO.COLOR_BACKGROUND, ips200pro.rgb888_to_rgb565(0xA5A5A5))
ips200pro.set_color(page_id2, IPS200PRO.COLOR_FOREGROUND, ips200pro.rgb888_to_rgb565(0x39C5BB))
ips200pro.set_color(page_id2, IPS200PRO.COLOR_BACKGROUND, ips200pro.rgb888_to_rgb565(0xB3B3B3))
ips200pro.set_color(page_id3, IPS200PRO.COLOR_FOREGROUND, ips200pro.rgb888_to_rgb565(0x39C5BB))
ips200pro.set_color(page_id3, IPS200PRO.COLOR_BACKGROUND, ips200pro.rgb888_to_rgb565(0xC7C7C7))
# 设置选中 页面 的标题字体色和背景色
ips200pro.set_color(page_id3, IPS200PRO.COLOR_PAGE_SELECTED_TEXT, 0xFFFF)
ips200pro.set_color(page_id3, IPS200PRO.COLOR_PAGE_SELECTED_BG  , 0x0000)
time.sleep_ms(500)

# 显示与隐藏 页面
# 页面 隐藏只是不显示页面内容 页面 标题还在
ips200pro.set_hidden(page_id3, True)
time.sleep_ms(500)
ips200pro.set_hidden(page_id3, False)
time.sleep_ms(500)

# 修改屏幕的亮度 通常在实例化屏幕对象后设置就行 想做待机降低屏幕亮度用它
ips200pro.set_backlight(50)
time.sleep_ms(500)
ips200pro.set_backlight(250)
time.sleep_ms(500)

# 修改屏幕的方向 通常在实例化屏幕对象后设置就行 想做重力感应可以加陀螺仪切换方向
ips200pro.set_dir(IPS200PRO.CROSSWISE)
time.sleep_ms(500)
ips200pro.set_dir(IPS200PRO.PORTRAIT)
time.sleep_ms(500)

while True:
    # 顺序切换页面

    # 需要注意的是 开启动画后需要 1s 的动画时间 视情况来决定是否开启动画
    ips200pro.page_switch(page_id1, IPS200PRO.PAGE_ANIM_ON)
    time.sleep_ms(2000)
    led.toggle()
    
    ips200pro.page_switch(page_id2)
    time.sleep_ms(500)
    led.toggle()
    
    ips200pro.page_switch(page_id3)
    time.sleep_ms(500)
    led.toggle()
    
    gc.collect()
