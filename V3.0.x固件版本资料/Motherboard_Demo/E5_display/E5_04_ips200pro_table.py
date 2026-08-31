
# 本示例程序演示如何使用 IPS200PRO 库
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的屏幕接口测试

# 本例程演示 IPS200PRO 屏幕的 表格 组件使用
# 关联的通用接口也会一并演示使用方法与效果

# 例程效果是屏幕显示 表格 并有一些变化
# 最终实现一个 表格 循环选择切换效果

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

ips200pro.help_table()

# 表格 组件接口 :
# ------------------------------------------------------------------------------
#   新建一个 表格 返回 表格 索引号
#   table_id = IPS200PRO.table_create(x, y, row, col)
#       x               横向坐标    |   必要参数 表格 的 X 轴坐标
#       y               竖向坐标    |   必要参数 表格 的 Y 轴坐标
#       row             表格行数    |   必要参数 表格行数
#       col             表格列数    |   必要参数 表格列数
#       return          返回内容    |   正常情况下返回对应 表格 的索引
# ------------------------------------------------------------------------------
#   通过对象调用 输出模块的 table 部分的使用帮助信息
#   IPS200PRO.help_table()
# ------------------------------------------------------------------------------
#   修改 表格 内容
#   IPS200PRO.table_string(table_id, row, col, str)
#       table_id        表格索引    |   必要参数 表格 的索引号
#       row             表格行数    |   必要参数 表格 行数
#       col             表格列数    |   必要参数 表格 列数
#       str             标签内容    |   必要参数 支持中文英文 UTF-8 格式
# ------------------------------------------------------------------------------
#   修改 表格 列宽
#   IPS200PRO.table_col_width(table_id, col, width)
#       table_id        表格索引    |   必要参数 表格 的索引号
#       col             表格列数    |   必要参数 表格 列数
#       width           列宽数值    |   必要参数 支持中文英文 UTF-8 格式
# ------------------------------------------------------------------------------
#   选中 表格 单元格或整行整列 需要注意的是 选中效果是所有表格组件共享 因此同一时间只能有一个表格设置选中
#   IPS200PRO.table_select(table_id, row, col)
#       table_id        表格索引    |   必要参数 表格 的索引号
#       row             表格行数    |   必要参数 表格 行数 为 0 时 为选择整列 超过表格行数时 取消选择
#       col             表格列数    |   必要参数 表格 列数 为 0 时 为选择整行 超过表格列数时 取消选择
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
#                                       TABLE 组件可用参数 xxx = (FOREGROUND, BACKGROUND, BORDER, TABLE_SELECTED_BG)
#                                       对应 (前景色, 背景色, 组件边线颜色, 表格选中后的颜色)
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

# 新建 表格 组件
table_id1 = ips200pro.table_create(  0,   0, 8, 2)
table_id2 = ips200pro.table_create(  0, 230, 1, 3)
table_id3 = ips200pro.table_create(  0, 260, 1, 2)
time.sleep_ms(500)

# 在指定的行列单元格位置显示内容
ips200pro.table_string(table_id1, 1, 1, "选项")
ips200pro.table_string(table_id1, 1, 2, "%s"%("数据"))
ips200pro.table_string(table_id1, 2, 1, "%s"%("PID"))
ips200pro.table_string(table_id1, 3, 1, "%s"%("SPD"))
ips200pro.table_string(table_id2, 1, 1, "%2.2f"%(1.75))
ips200pro.table_string(table_id2, 1, 2, "%2.2f"%(6.50))
ips200pro.table_string(table_id2, 1, 3, "%2.2f"%(0.25))
ips200pro.table_string(table_id3, 1, 1, "%d"%(102))
ips200pro.table_string(table_id3, 1, 2, "%d"%(99))
time.sleep_ms(500)

# 设置指定列的宽度
ips200pro.table_col_width(table_id1, 1,  60)
ips200pro.table_col_width(table_id2, 1,  60)
ips200pro.table_col_width(table_id2, 2,  60)
ips200pro.table_col_width(table_id2, 3,  60)
time.sleep_ms(500)
ips200pro.table_col_width(table_id1, 2, 180)
ips200pro.table_col_width(table_id3, 1,  90)
ips200pro.table_col_width(table_id3, 2,  90)
time.sleep_ms(500)

# 选择指定行列单元格 指定列指定行 以及取消选择
ips200pro.table_select(table_id1, 2, 1)
time.sleep_ms(500)
ips200pro.table_select(table_id1, 3, 2)
time.sleep_ms(500)
ips200pro.table_select(table_id1, 0, 2)
time.sleep_ms(500)
ips200pro.table_select(table_id1, 2, 0)
time.sleep_ms(500)
ips200pro.table_select(table_id1, 255, 255)
time.sleep_ms(500)

# 重新设置 表格 的字体 它会立即生效
ips200pro.set_font(table_id1, IPS200PRO.FONT_SIZE_24)
ips200pro.set_font(table_id2, IPS200PRO.FONT_SIZE_24)
ips200pro.set_font(table_id3, IPS200PRO.FONT_SIZE_24)
time.sleep_ms(500)
ips200pro.set_font(table_id1, IPS200PRO.FONT_SIZE_16)
ips200pro.set_font(table_id2, IPS200PRO.FONT_SIZE_16)
ips200pro.set_font(table_id3, IPS200PRO.FONT_SIZE_16)
time.sleep_ms(500)

# 修改 表格 的 前景色 背景色 边线颜色 表格选中后的颜色
ips200pro.set_color(table_id1, IPS200PRO.COLOR_FOREGROUND       , ips200pro.rgb888_to_rgb565(0xF5F8FF))
ips200pro.set_color(table_id1, IPS200PRO.COLOR_BACKGROUND       , ips200pro.rgb888_to_rgb565(0xA6BDFB))
ips200pro.set_color(table_id1, IPS200PRO.COLOR_BORDER           , ips200pro.rgb888_to_rgb565(0xD4DDFD))
ips200pro.set_color(table_id1, IPS200PRO.COLOR_TABLE_SELECTED_BG, ips200pro.rgb888_to_rgb565(0xFDAB76))
ips200pro.set_color(table_id2, IPS200PRO.COLOR_FOREGROUND       , ips200pro.rgb888_to_rgb565(0xF5F8FF))
ips200pro.set_color(table_id2, IPS200PRO.COLOR_BACKGROUND       , ips200pro.rgb888_to_rgb565(0xA6BDFB))
ips200pro.set_color(table_id2, IPS200PRO.COLOR_BORDER           , ips200pro.rgb888_to_rgb565(0xD4DDFD))
ips200pro.set_color(table_id2, IPS200PRO.COLOR_TABLE_SELECTED_BG, ips200pro.rgb888_to_rgb565(0xFDAB76))
ips200pro.set_color(table_id3, IPS200PRO.COLOR_FOREGROUND       , ips200pro.rgb888_to_rgb565(0xF5F8FF))
ips200pro.set_color(table_id3, IPS200PRO.COLOR_BACKGROUND       , ips200pro.rgb888_to_rgb565(0xA6BDFB))
ips200pro.set_color(table_id3, IPS200PRO.COLOR_BORDER           , ips200pro.rgb888_to_rgb565(0xD4DDFD))
ips200pro.set_color(table_id3, IPS200PRO.COLOR_TABLE_SELECTED_BG, ips200pro.rgb888_to_rgb565(0xFDAB76))
time.sleep_ms(500)

# 重新设置 表格 的位置
ips200pro.set_position(table_id1, 0, 24)
time.sleep_ms(500)

# 显示与隐藏 表格
ips200pro.set_hidden(table_id1, True)
time.sleep_ms(500)
ips200pro.set_hidden(table_id1, False)
time.sleep_ms(500)

# 设置 表格 的依附关系 可以实现 表格 嵌套的视觉效果
# 当设置了依附效果后 依附的组件就会跟随被依附的组件 例如修改被依附组件位置 依附组件也会随动
ips200pro.set_parent(table_id2, table_id1)
time.sleep_ms(500)
ips200pro.set_position(table_id2,  59,  23)
time.sleep_ms(500)
ips200pro.set_parent(table_id3, table_id1)
time.sleep_ms(500)
ips200pro.set_position(table_id3,  59,  46)
time.sleep_ms(500)

# 设置 表格 的依附关系 将其切换到另一个 页面 下
ips200pro.set_parent(table_id1, page_id1)
time.sleep_ms(500)

# 切换页面 可以看到刚刚操作的 表格 已经切换过来了
# 需要注意的是 开启动画后需要 1s 的动画时间 视情况来决定是否开启动画
ips200pro.page_switch(page_id1, IPS200PRO.PAGE_ANIM_ON)
time.sleep_ms(1000)

# 重新设置 表格 的位置
ips200pro.set_position(table_id1,   0,  50)
time.sleep_ms(500)

while True:
    
    for col in range(1, 3, 1):
        for row in range(1, 9, 1):
            ips200pro.table_select(table_id1, row, col)
            time.sleep_ms(500)
            led.toggle()
    
    gc.collect()
