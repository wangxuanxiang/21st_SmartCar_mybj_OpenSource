import sensor
import image
import time
import os
from pyb import LED

# ======================== 常量定义（与从车一致）=======================
# 摄像头配置
CAMERA_PIXFORMAT = sensor.RGB565
CAMERA_FRAMESIZE = sensor.QQVGA  # 160x120
CAMERA_FRAMERATE = 60
CAMERA_BRIGHTNESS = 600

# 拍照参数
SKIP_FRAMES = 100      # 等待摄像头稳定的帧数
PHOTO_COUNT = 10       # 拍照张数
SAVE_DIR = '/sd/sample'

# ======================== 初始化 ========================
# 指示灯（LED 3 为蓝色）
blue_led = LED(3)

# 摄像头初始化（从车：不倒转）
sensor.reset()
sensor.set_pixformat(CAMERA_PIXFORMAT)
sensor.set_framesize(CAMERA_FRAMESIZE)
sensor.set_framerate(CAMERA_FRAMERATE)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_brightness(CAMERA_BRIGHTNESS)
sensor.set_contrast(2)
sensor.skip_frames(time=200)
clock = time.clock()

# 创建保存目录
try:
    os.mkdir(SAVE_DIR)
except OSError:
    pass  # 目录已存在则忽略

# ======================== 主程序 ========================
print("等待摄像头稳定...")
for i in range(SKIP_FRAMES):
    clock.tick()
    img = sensor.snapshot()
    print(f"跳过帧 {i+1}/{SKIP_FRAMES}", end='\r')

print("\n准备拍照（蓝灯闪一下）")
blue_led.on()
time.sleep_ms(200)
blue_led.off()
time.sleep_ms(200)

for i in range(PHOTO_COUNT):
    clock.tick()
    img = sensor.snapshot()

    filename = f"{SAVE_DIR}/photo_{i+1:02d}.jpg"
    img.save(filename)
    print(f"保存: {filename}  fps={clock.fps():.1f}")
    time.sleep_ms(100)

print(f"\n拍照完成，共保存 {PHOTO_COUNT} 张到 {SAVE_DIR}/")

# 等待一秒后蓝灯再闪一下
time.sleep_ms(1000)
blue_led.on()
time.sleep_ms(200)
blue_led.off()
