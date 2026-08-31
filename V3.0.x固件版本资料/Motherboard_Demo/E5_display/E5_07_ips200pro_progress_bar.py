
# 本示例程序演示如何使用 IPS200PRO 库
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的屏幕接口测试

# 本例程演示 IPS200PRO 屏幕的 进度条 组件使用
# 关联的通用接口也会一并演示使用方法与效果

# 例程效果是屏幕显示 进度条 并有一些变化
# 最终实现一个动画效果

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

ips200pro.help_progress_bar()

# 进度条 组件接口 :
# ------------------------------------------------------------------------------
#   新建一个 进度条 返回 进度条 索引号
#   progress_bar_id = IPS200PRO.progress_bar_create(x, y, width, height)
#       x               横向坐标    |   必要参数 进度条 的 X 轴坐标
#       y               竖向坐标    |   必要参数 进度条 的 Y 轴坐标
#       width           组件宽度    |   必要参数 进度条 的宽度
#       height          组件高度    |   必要参数 进度条 的高度
#       return          返回内容    |   正常情况下返回对应 进度条 的索引
# ------------------------------------------------------------------------------
#   通过对象调用 输出模块的 progress_bar 部分的使用帮助信息
#   IPS200PRO.help_progress_bar()
# ------------------------------------------------------------------------------
#   修改 进度条 数值 通过设置起始和停止位置来设置进度条的位置
#   IPS200PRO.progress_bar_value(progress_bar_id, start_value, end_value)
#       progress_bar_id 进度条值    |   必要参数 进度条 的索引号
#       start_value     起始数值    |   必要参数 正整数
#       end_value       结束数值    |   必要参数 正整数
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
#                                       PROGRESS BAR 组件可用参数 xxx = (FOREGROUND, BACKGROUND)
#                                       对应 (前景色, 背景色)
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

# 新建 进度条 组件
progress_bar_id1 = ips200pro.progress_bar_create( 20, 100, 200, 30)
progress_bar_id2 = ips200pro.progress_bar_create( 70, 140, 100, 10)
progress_bar_id3 = ips200pro.progress_bar_create( 95, 160, 50, 30)
time.sleep_ms(500)

# 修改 进度条 的位置
ips200pro.progress_bar_value(progress_bar_id1,  50,  75)
ips200pro.progress_bar_value(progress_bar_id2,  38,  63)
ips200pro.progress_bar_value(progress_bar_id3,  25,  50)
time.sleep_ms(500)

# 修改 进度条 的 前景色 背景色
ips200pro.set_color(progress_bar_id1, IPS200PRO.COLOR_FOREGROUND, ips200pro.rgb888_to_rgb565(0x39C5BB))
ips200pro.set_color(progress_bar_id1, IPS200PRO.COLOR_BACKGROUND, ips200pro.rgb888_to_rgb565(0x5A5A5A))
ips200pro.set_color(progress_bar_id2, IPS200PRO.COLOR_FOREGROUND, 0xFFFF)
ips200pro.set_color(progress_bar_id2, IPS200PRO.COLOR_BACKGROUND, 0x001F)
ips200pro.set_color(progress_bar_id3, IPS200PRO.COLOR_FOREGROUND, 0xF800)
ips200pro.set_color(progress_bar_id3, IPS200PRO.COLOR_BACKGROUND, 0x0000)
time.sleep_ms(500)

# 重新设置 进度条 的位置
ips200pro.set_position(progress_bar_id1, 20,  70)
time.sleep_ms(500)
ips200pro.set_position(progress_bar_id1, 20, 100)
time.sleep_ms(500)
ips200pro.set_position(progress_bar_id3, 95, 190)
time.sleep_ms(500)
ips200pro.set_position(progress_bar_id3, 95, 160)
time.sleep_ms(500)

# 显示与隐藏 进度条
ips200pro.set_hidden(progress_bar_id1, True)
time.sleep_ms(500)
ips200pro.set_hidden(progress_bar_id2, True)
time.sleep_ms(500)
ips200pro.set_hidden(progress_bar_id3, True)
time.sleep_ms(500)
ips200pro.set_hidden(progress_bar_id1, False)
time.sleep_ms(500)
ips200pro.set_hidden(progress_bar_id2, False)
time.sleep_ms(500)
ips200pro.set_hidden(progress_bar_id3, False)
time.sleep_ms(500)

# 设置 进度条 的依附关系 将其切换到另一个 页面 下
ips200pro.set_parent(progress_bar_id1, page_id1)
time.sleep_ms(500)

# 切换页面 可以看到刚刚操作的 进度条 已经切换过来了
# 需要注意的是 开启动画后需要 1s 的动画时间 视情况来决定是否开启动画
ips200pro.page_switch(page_id1, IPS200PRO.PAGE_ANIM_ON)
time.sleep_ms(1000)

# 将 进度条 依次切换过来 设置位置
ips200pro.set_position(progress_bar_id1, 20, 100)
time.sleep_ms(500)
ips200pro.set_parent(progress_bar_id2, page_id1)
time.sleep_ms(500)
ips200pro.set_position(progress_bar_id2, 70, 140)
time.sleep_ms(500)
ips200pro.set_parent(progress_bar_id3, page_id1)
time.sleep_ms(500)
ips200pro.set_position(progress_bar_id3, 95, 160)
time.sleep_ms(500)

# 生成 RGB565 格式的七彩循环渐变色元组
#   colors_rgb565_tuple = generate_rainbow_colors_rgb565(lenght)
#   lenght          元组长度    |   必要参数 生成的渐变色元组长度
def generate_rainbow_colors_rgb565 (num_colors):
    colors = []
    # 将颜色循环分为6个阶段：红→黄→绿→青→蓝→紫→红
    segment_length = num_colors // 6
    
    for i in range(num_colors):
        # 确定当前颜色在哪个阶段
        segment = i // segment_length
        pos_in_segment = (i % segment_length) / segment_length
        
        # 根据阶段计算RGB值
        # 红→黄 (255,   0,   0) → (255, 255,   0)
        # 黄→绿 (255, 255,   0) → (  0, 255,   0)
        # 绿→青 (  0, 255,   0) → (  0, 255, 255)
        # 青→蓝 (  0, 255, 255) → (  0,   0, 255)
        # 蓝→紫 (  0,   0, 255) → (255,   0, 255)
        # 紫→红 (255,   0, 255) → (255,   0,   0)
        if   segment == 0:  r, g, b = 255, int(255 * pos_in_segment), 0
        elif segment == 1:  r, g, b = int(255 * (1 - pos_in_segment)), 255, 0
        elif segment == 2:  r, g, b = 0, 255, int(255 * pos_in_segment)
        elif segment == 3:  r, g, b = 0, int(255 * (1 - pos_in_segment)), 255
        elif segment == 4:  r, g, b = int(255 * pos_in_segment), 0, 255
        else:               r, g, b = 255, 0, int(255 * (1 - pos_in_segment))
        
        # 将RGB888转换为RGB565
        # R: 5位 (取高5位), G: 6位 (取高6位), B: 5位 (取高5位)
        r5 = (r >> 3) & 0x1F
        g6 = (g >> 2) & 0x3F
        b5 = (b >> 3) & 0x1F
        
        # 组合成16位RGB565值 (R在高位，B在低位)
        rgb565 = (r5 << 11) | (g6 << 5) | b5
        colors.append(rgb565)
    
    return tuple(colors)

# 由于按照色阶分成了六个阶段循环
# R > R+G > G > G+B > B > B+R
# 红 > 黄 > 绿 > 青 > 蓝 > 紫
# 也就是 RGB 按顺序混合过渡
# 所以颜色按照六的倍数来生成会更顺滑
# 否则会有细微断层
color_max = 6 * 50
colors_list = generate_rainbow_colors_rgb565(color_max)

step_dir = 1
step_value = 25
color_index = 0

while True:
    time.sleep_ms(20)
    
    color_index = (0) if(color_max - 1 == color_index) else (color_index + 1)
    step_value = step_value + step_dir
    if step_value == 50:   step_dir = -1
    if step_value == 25:   step_dir =  1
    
    ips200pro.progress_bar_value(progress_bar_id1,  (75 - step_value),  (100 - step_value))
    ips200pro.progress_bar_value(progress_bar_id3,  step_value,  step_value + 25)
    ips200pro.set_color(progress_bar_id3, IPS200PRO.COLOR_FOREGROUND, colors_list[color_index])
    
    led.value((step_dir == 1))
    gc.collect()
