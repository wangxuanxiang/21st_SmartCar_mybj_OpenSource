import sensor
import time

# ======================== 亮度调试工具 ========================
# 固定摄像头亮度600下，循环打印图像亮度均值 l_mean（0-100）。
#
# 用途：标定 main_final_single.py / slave_final_single.py 中的
#       自适应亮度分档阈值。摄像头配置与主程序保持一致，测出来的值
#       可直接对照三档：
#           l_mean 0-35   → 暗   → 模型亮度1200
#           l_mean 36-90  → 正常 → 模型亮度800
#           l_mean 91-100 → 亮   → 模型亮度400

# 摄像头配置（与 main_final_single.py 一致）
CAMERA_PIXFORMAT = sensor.RGB565
CAMERA_FRAMESIZE = sensor.QQVGA  # 160x120
CAMERA_FRAMERATE = 60
CAMERA_BRIGHTNESS = 600  # 固定600，测该亮度下的 l_mean

sensor.reset()
sensor.set_pixformat(CAMERA_PIXFORMAT)
sensor.set_framesize(CAMERA_FRAMESIZE)
sensor.set_framerate(CAMERA_FRAMERATE)
sensor.set_auto_gain(False)      # 关闭自动增益，保证亮度可控
sensor.set_auto_whitebal(False)  # 关闭自动白平衡
sensor.set_brightness(CAMERA_BRIGHTNESS)
sensor.set_contrast(2)           # 与主程序一致
sensor.skip_frames(time=2000)    # 跳过初始帧，让摄像头稳定

while True:
    stats = sensor.snapshot().get_statistics()
    l_mean = stats.l_mean()
    print("l_mean =", l_mean)    # 0-100，暗环境低、亮环境高
    time.sleep_ms(200)           # 200ms打印一次，方便观察
