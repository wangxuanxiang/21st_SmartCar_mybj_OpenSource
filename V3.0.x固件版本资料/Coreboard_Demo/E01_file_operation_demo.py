
# 本示例程序演示如何使用 os 和 io 库
# 使用 RT1021-MicroPython 核心板进行测试

# 示例程序运行效果为打开或新建一个 user_data.txt 文件进行文件读写
# 并通过 RT1021-MicroPython 核心板的 Type-C 的 CDC 虚拟串口输出文件操作结果

# 包含 os 与 io 类
import os
import io

# 切换到 /flash 目录
os.chdir("/flash")
try:
    # 通过 try 尝试打开文件 因为 r+ 读写模式不会新建文件
    print("尝试打开 user_data.txt 文件.")
    user_file = io.open("user_data.txt", "r+")
except:
    # 如果打开失败证明没有这个文件 所以使用 w+ 读写模式新建文件
    print("user_data.txt 文件不存在，新建该文件.")
    user_file = io.open("user_data.txt", "w+")

# 不要在循环里去反复 打开/关闭 文件
# 不要在循环里去反复 打开/关闭 文件
# 不要在循环里去反复 打开/关闭 文件

# w+ 读写模式可以打开原有文件也可以新建文件 为什么不直接使用 w+ 读写模式打开文件
# 因为用 w+ 模式打开的文件会强制从头覆盖原数据写入 这会导致原有数据丢失

# 将指针移动到文件头 读取三行数据显示 也就是显示原有数据
user_file.seek(0,0)
print(user_file.readline())
print(user_file.readline())
print(user_file.readline())

# 将指针移动到文件头 0 偏移的位置 这个函数参照 Python 的 File 模块
user_file.seek(0,0)
# 使用 write 方法写入数据到缓冲区 这里写入整形数、浮点数和字符串
user_file.write("%d\n%.1f\n%s"%(33,11.1,'Hello'))
# 将缓冲区数据写入到文件 清空缓冲区 相当于保存指令
user_file.flush()

# 不要在循环里去反复写入擦除文件 Flash 是有寿命的 禁不起这种折腾
# 不要在循环里去反复写入擦除文件 Flash 是有寿命的 禁不起这种折腾
# 不要在循环里去反复写入擦除文件 Flash 是有寿命的 禁不起这种折腾

# 将指针重新移动到文件头
user_file.seek(0,0)
# 读取三行数据 因为刚刚写入了三行
print(user_file.readline())
print(user_file.readline())
print(user_file.readline())

# 将指针重新移动到文件头
user_file.seek(0,0)
# 读取三行数据 到临时变量 分别强制转换回各自类型
data1 = int(user_file.readline())
data2 = float(user_file.readline())
data3 = str(user_file.readline())
# 将数据重新输出 这就演示了如何保存数据和读取数据
print(data1)
print(data2)
print(data3)

# 用整形数与浮点数相加得到浮点数
print(data1 + data2)
# 强制转换结果浮点数为整形数
print(int(data1 + data2))
# 结果是数据计算

# 最后将文件关闭即可
user_file.close()
