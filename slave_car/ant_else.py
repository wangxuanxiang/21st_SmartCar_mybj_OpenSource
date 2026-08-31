from micropython import const
import time
import gc
# 引入 VL53L4CD 驱动
from vl53l4cd import RANGE_VALID

PI = const(3.1415926)
# 计数器
counter = 0 
##############################【蜂鸣器】##############################
BEEP_OFF = const(0)
BEEP_ON = const(1)
class beep:
    def __init__(self, beep):
        # 注入蜂鸣器对象
        self.beep = beep
        self.beep_state = BEEP_OFF

        gc.collect()

    # 蜂鸣器警告函数(响3声，每500ms响一声，每次持续50ms)
    def beep_warn(self) -> None:
        if self.beep_state == BEEP_OFF:
            self.beep_state = BEEP_ON
            for i in range(3):
                time.sleep_ms(50)
                self.beep.high()
                time.sleep_ms(50)
                self.beep.low()
                time.sleep_ms(200)
                self.beep_state = BEEP_OFF
            return 
        elif self.beep_state == BEEP_ON:
            return 

    # 低电量警告函数(响5声，每50ms响一声，每次持续50ms)
    def low_power_warn(self) -> None:
        if self.beep_state == BEEP_OFF:
            self.beep_state = BEEP_ON
            for i in range(5):
                time.sleep_ms(50)
                self.beep.high()
                time.sleep_ms(50)
                self.beep.low()
                time.sleep_ms(50)
                self.beep_state = BEEP_OFF
            return 
        elif self.beep_state == BEEP_ON:
            return

    # 未发现tof
    def failure_to_find_tof(self) -> None:
        if self.beep_state == BEEP_OFF:
            self.beep_state = BEEP_ON
            for i in range(5):
                time.sleep_ms(50)
                self.beep.high()
                time.sleep_ms(300)
                self.beep.low()
                time.sleep_ms(100)
                self.beep_state = BEEP_OFF
            return 
        elif self.beep_state == BEEP_ON:
            return
        
    # 按键测试函数(响一声，持续80ms)
    def key_test(self) -> None:
        if self.beep_state == BEEP_OFF:
            self.beep_state = BEEP_ON
            self.beep.high()
            time.sleep_ms(80)
            self.beep.low()
            self.beep_state = BEEP_OFF
            return
        elif self.beep_state == BEEP_ON:
            return

    # 蜂鸣器测试函数(响一声，持续50ms)
    def test(self) -> None:
        if self.beep_state == BEEP_OFF:
            self.beep_state = BEEP_ON
            self.beep.high()
            time.sleep_ms(50)
            self.beep.low()
            self.beep_state = BEEP_OFF
            return
        elif self.beep_state == BEEP_ON:
            return


##############################【uart串口解析数据】##############################
# 指令管理类
class order_manager:
    def __init__(self, flash_sys, uart):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入串口对象
        self.my_uart = uart

        # 使用模型还是色块
        self.if_model = self.flash_sys.find_value("if_model")
    
    # 切换到目标识别模式（模型?
    def mode_target(self):
        if self.if_model:
            self.my_uart.write("M")
        else:
            self.my_uart.write("C")

    def send_object_kind(self, object_kind):
        self.my_uart.write(object_kind.lower())

    def mode_all_detect(self):
        self.my_uart.write("A")

    # 当前模式结束
    def finish(self):
        self.my_uart.write("F")    
        
# 状态机解析串口数据类
class UARTProtocol:
    def __init__(self, uart):
        # 注入串口对象
        self.my_uart = uart
        self.state_coordinate = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待x, 3:等待y, 4:等待物体种类, 5:等待帧尾
        self.state_apriltag = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待x, 3:等待y, 4:等待距离低8位, 5:等待距离高8位, 6:等待帧尾
        self.coordinate_buffer = [0, 0, 0, 0, '', 0]
        self.apriltag_buffer = [0, 0, 0, 0, 0, 0, 0]
        self.byte_count = 0
        self.state_detect_all_objects = 0 # 0:等待帧头1, 1:等待物体数量, 2:等待发送物体讯息, 5:等待帧尾
        self.detect_buffer = [0,[]]
        self.object_buffer = ['',0,0]
        self.state_object = 0 # 0:等待物体种类, 1:等待x, 2:等待y, 3:等待帧尾
        gc.collect()
    def clear_uart_buffer(self):
        self.state_coordinate = 0
        self.state_apriltag = 0
        self.state_detect_all_objects = 0
        self.state_object = 0
        self.detect_buffer = [0,[]]
        self.coordinate_buffer = [0, 0, 0, 0, '', 0]
        self.object_buffer = ['',0,0]
        self.my_uart.read(self.my_uart.any())#清空缓冲区
    # 非阻塞接收并解析物体中心的像素点坐标  
    def coordinate_receive(self):
        last_valid_frame = None
        # 持续读取直到处理完当前缓冲区的所有数据
        while self.my_uart.any():	
            byte = self.my_uart.read(1)[0]
            
            if self.state_coordinate == 0:
                if byte == 0xA5:
                    self.state_coordinate = 1
            elif self.state_coordinate == 1:
                if byte == 0xA6:
                    self.state_coordinate = 2
                else:
                    self.state_coordinate = 0
            elif self.state_coordinate == 2:
                self.coordinate_buffer[2] = byte
                self.state_coordinate = 3
            elif self.state_coordinate == 3:
                self.coordinate_buffer[3] = byte
                self.state_coordinate = 4
            elif self.state_coordinate == 4:
                self.coordinate_buffer[4] = byte
                self.state_coordinate = 5
            elif self.state_coordinate == 5:
                if byte == 0x5B:
                    # 解析成功，保存当前帧，但【不要】清空缓冲区，【不要】立即返回
                    x, y = self.coordinate_buffer[2], self.coordinate_buffer[3]
                    last_valid_frame = (x, y, self.coordinate_buffer[4])
                    self.state_coordinate = 0 
                else:
                    self.state_coordinate = 0
                
        # 循环结束后，返回缓冲区里最新的一帧
        return last_valid_frame
    def reset_detect_objects(self):
        self.state_detect_all_objects = 0
        self.state_object = 0
        self.detect_buffer = [0,[]]
        self.object_buffer = ['',0,0]
    def detect_objects_on_the_court(self):
        objects_package = None
        while self.my_uart.any():
            byte = self.my_uart.read(1)[0]
            if self.state_detect_all_objects == 0:
                self.reset_detect_objects()
                if byte == 0x77:
                    self.state_detect_all_objects = 1
            elif self.state_detect_all_objects == 1:#等待物体数量
                if byte>0x00 and byte<=0x10:#物体数量大于0小于等于16
                    self.detect_buffer[0]=byte
                    self.state_detect_all_objects = 2
                else:
                    self.reset_detect_objects()
                    continue
            elif self.state_detect_all_objects == 2:
                if self.state_object == 0:
                    if byte in [84,69,83,66,87]:
                        self.state_object = 1
                        self.object_buffer[0] = byte
                    else:
                        self.reset_detect_objects()
                        continue
                elif self.state_object == 1:
                    self.object_buffer[1] = byte
                    self.state_object = 2
                elif self.state_object == 2:
                    self.object_buffer[2] = byte
                    self.state_object = 3
                elif self.state_object == 3:
                    if byte == 0x5B:
                        self.state_object = 0
                        self.detect_buffer[1].append(self.object_buffer[:])
                        if len(self.detect_buffer[1])>=self.detect_buffer[0]:
                            self.state_detect_all_objects = 3
                    else:
                        self.reset_detect_objects()
                        continue
            elif self.state_detect_all_objects == 3:
                if byte == 0x78:
                    objects_package = self.detect_buffer
                self.reset_detect_objects()
                continue
        return objects_package
    def send_object_kind(self, object_kind):
        self.my_uart.write(object_kind.lower())

# 主从机通信类
class LinkProtocol:
    def __init__(self, uart3):
        # 注入串口对象
        self.my_uart3 = uart3
        # 当前要伺服的物体
        self.aimed_object = ''
        # 创建字节流缓冲区
        self.raw_buffer = b''           
        self.max_buf = 128         # 缓冲区最大长度，防止内存泄漏
        self.start_idx = 0          # 上次成功解析后剩余数据的起始索引（相对于raw_buffer）
        self.end_idx = 0            # 上次成功解析后剩余数据的结束索引（相对于raw_buffer）

        gc.collect()

    # 用于从车向主车发送当前状态数据的接口
    def send_slave_state(self, state):
        if state == "ready":
            self.my_uart3.write('R'.encode('utf-8'))
        elif state == "lost":
            self.my_uart3.write('L'.encode('utf-8'))
        elif state == "finish":
            self.my_uart3.write('F'.encode('utf-8'))
        elif state == "get":
            self.my_uart3.write('G'.encode('utf-8'))

    # 用于从车解析主车发送的开始信号
    def get_start_signal(self):
        if self.my_uart3.any():
            try:
                byte = self.my_uart3.read(1)[0]
                if byte == ord('S'):
                    return True
            except:
                pass
        return False
        
    # 解析主车发送的路径信息
    def get_path_list(self):
            """
            解析主车发送的任务路径包
            发送格式: #T,90.0,120.5,80.1!  (或 #S, #B, #P, #E, #W，#M，#A，#R，中间包含转向角度等)
            :return: 成功返回 [task_type, target_turn, (x, y)], 如 ['T', 90.0, (120.5, 80.1)]
                    失败返回 None
            """
            # 1. 填充缓冲区 (保持原样)
            if self.my_uart3.any():
                try:
                    chunk = self.my_uart3.read()
                    if chunk:
                        self.raw_buffer += chunk
                except:
                    pass 
            
            if not self.raw_buffer:
                return None
            
            # 2. 内存保护 (保持原样)
            if len(self.raw_buffer) > self.max_buf:
                self.raw_buffer = self.raw_buffer[-self.max_buf:]

            # 3. 寻找包尾 '!'
            self.end_idx = self.raw_buffer.find(b'!')
            if self.end_idx == -1:
                return None 

            # 4. 寻找包头 (核心修改点：支持多种包头)
            # 逻辑：先找 '#'，再判断后面是不是符合 T, S, B 格式
            self.start_idx = self.raw_buffer.find(b'#', 0, self.end_idx)

            if self.start_idx == -1:
                # 没找到包头，清理掉无效的尾部之前的数据
                self.raw_buffer = self.raw_buffer[self.end_idx+1:]
                return None

            # 检查包头格式是否完整（# 后面至少要有 "X," 三个字节）
            if self.start_idx + 2 >= self.end_idx:
                # 数据不完整，继续等待
                return None

            # 提取标识符 (T, S, B 或 P)
            try:
                # 这里的 tag_type 是 bytes 类型，转成 string 方便后续判断
                tag_type = self.raw_buffer[self.start_idx + 1 : self.start_idx + 2].decode('utf-8')
                
                # 如果不是我们预期的指令，说明可能是脏数据
                if tag_type not in ['T', 'S', 'B', 'P', 'E', 'W', 'M', 'A', 'R']:
                    # 这种情况下，丢弃这个错误的开头，继续找下一个
                    self.raw_buffer = self.raw_buffer[self.start_idx + 1:]
                    return None
                    
            except:
                return None
            payload_bytes = self.raw_buffer[self.start_idx + 3 : self.end_idx]
            # 6. 消费缓冲区
            self.raw_buffer = self.raw_buffer[self.end_idx+1:]
            # 7. 纯字符串解析逻辑
            try:
                payload_str = payload_bytes.decode('utf-8')
                # 按照逗号分割，应该得到比如 ['L', '120.5', '80.1'] 这样的3个元素
                parts = payload_str.split(',')
                if len(parts) == 3:
                    target_turn = float(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    # 【关键点】返回类型：物体种类，转向，目标坐标
                    return [tag_type, target_turn, (x, y)]
                else:
                    return None
            except Exception as e:
                return None

##############################【flash系统操作】##############################
class flash_system:
    def __init__(self, beep, file_path: str):
        # 注入蜂鸣器对象，用于警报
        self.beep = beep
        # 传入文件路径
        self.file_path = file_path  # type: str
        # 创建变量字典
        self.config = dict()
        gc.collect()
    # 将字符串解析为整数或浮点数，如果无法解析则返回原始字符串
    def phase_num_string(self, s: str):
        # 尝试解析为整数(只支持十进制)
        try:
            value = int(s, 10)
            return value
        except ValueError:
            pass
        # 尝试解析为浮点数
        try:
            value = float(s)
            return value
        except ValueError:
            pass
        # 如果无法解析为数字，则返回原始字符串
        return s
    def _parse_tuple_list(self, s: str):
        items = []
        current = []
        token = ''
        in_tuple = False
        for ch in s:
            if ch == '(':
                in_tuple = True
                current = []
                token = ''
            elif ch == ')':
                if token.strip():
                    current.append(float(token.strip()))
                items.append(tuple(current))
                in_tuple = False
                token = ''
            elif ch == ',':
                if in_tuple and token.strip():
                    current.append(float(token.strip()))
                    token = ''
            elif in_tuple:
                token += ch
        return items
    
    # 打开参数文件并进行解析，传入一个文件路径，返回一个字典
    def phase_config(self) -> None:
        try:
            f = open(self.file_path, 'r')
        except FileNotFoundError as e:
            print(e)
            print(f"Error: File {self.file_path} not found.")
            return
        line_count = 0
        try:
            line_iter = f
        except:
            line_iter = []
        for line in line_iter:
            # 跳过空行和注释行
            if not line or line.startswith('#') or line.startswith('\r\n'):
                continue
            line = line.strip()
            if '=' not in line:
                continue
            line = line.split('=', 1)
            var_name = line[0].strip()
            var_value = line[1].strip()
            if not var_value:
                continue
            # 解析变量值
            # 如果值以双引号开头和结尾，认为它是一个列表的字符串表示，替换为方括号后使用eval解析为列表
            if var_value[0] == '"' and var_value[-1] == '"':  # 列表类型
                try:
                    inner = var_value[1:-1]
                    if not inner.strip():
                        self.config[var_name] = []
                    elif '(' in inner:
                        self.config[var_name] = self._parse_tuple_list(inner)
                    else:
                        self.config[var_name] = [float(x.strip()) for x in inner.split(',')]
                except Exception as e:
                    print(f"Error: Failed to parse {var_name} = {var_value}")
                    self.beep.beep_warn()
            else:
                self.config[var_name] = self.phase_num_string(var_value)
            line_count += 1
            if line_count % 8 == 0:
                gc.collect()
        f.close()

    def release_config(self) -> None:
        # Deleting the reference releases the hash table and all unused values.
        # dict.clear() may retain the allocated hash table.
        if hasattr(self, "config"):
            del self.config
        gc.collect()
        
    def find_value(self, var_name: str):
        try:
            var_value = self.config[var_name.strip()]
            return var_value
        except KeyError as e:
            print(f"Failure to find {var_name.strip()} in {self.file_path}!")
            self.beep.beep_warn()
            return 0
        
    def check_list_format(self) -> None:
        """检查特定的列表与其内部变量格式是否正确"""
        error_flag = False

        # 检查 cube_obstacles: [(float, float, float, float), ...]
        if "cube_obstacles" in self.config:
            co = self.config["cube_obstacles"]
            if type(co) is not list:
                print("Error: 'cube_obstacles' 必须是列表")
                error_flag = True
            else:
                for obs in co:
                    if type(obs) is not tuple or len(obs) != 4 or not all(type(x) in (int, float) for x in obs):
                        print("Error: 'cube_obstacles' 的元素格式错误，应为包干4个数字的元组 (float, float, float, float)")
                        error_flag = True
                        break

        # 检查 circle: [(float, float), ...]
        if "circle" in self.config:
            cr = self.config["circle"]
            if type(cr) is not list:
                print("Error: 'circle' 必须是列表")
                error_flag = True
            else:
                for cir in cr:
                    if type(cir) is not tuple or len(cir) != 2 or not all(type(x) in (int, float) for x in cir):
                        print("Error: 'circle' 的元素格式错误，应为包干2个数字的元组 (float, float)")
                        error_flag = True
                        break
        if error_flag:
            self.beep.beep_warn()

class TofControl:
    def __init__(self, flash_sys, beep, car, plan, dist_pid_L, dist_pid_R, tof_L = None, tof_R = None):
        self.flash_sys = flash_sys
        self.my_beep = beep
        self.tof_L = tof_L
        self.tof_R = tof_R
        self.my_car = car
        self.my_plan = plan
        self.dist_pid_L = dist_pid_L
        self.dist_pid_R = dist_pid_R

        # 传感器读数
        self.data_L = -1.0
        self.data_R = -1.0
        self.status_L = RANGE_VALID
        self.status_R = RANGE_VALID 
        self.bias_L = 0.0  # 左传感器偏置值
        self.bias_R = 0.0  # 右传感器偏置值
        self.car_dist_increment = self.flash_sys.find_value("car_dist_increment")
        # 传感器读数合理的最小值和最大值
        self.tof_valid_min = self.flash_sys.find_value("tof_valid_min")  # type: float
        self.tof_valid_max = self.flash_sys.find_value("tof_valid_max")  # type: float

        self.if_init_fil = False # 是否初始化滤波器
        self.which_one = None  # 当前使用的传感器，'L'表示左传感器，'R'表示右传感器
        self._stale_count_L = 0  # 左传感器连续无效计数
        self._stale_count_R = 0  # 右传感器连续无效计数
        self._invalid_count = 0  # 数据超出valid范围的连续计数
        # I2C只能由主循环service()访问，定时器中的控制逻辑只读取缓存。
        self._desired_side = None
        self._active_side = None
        self._sync_retry_ms = 0
        self._next_read_ms = 0
        self._tof_period_ms = 10
        self._error_count_L = 0
        self._error_count_R = 0
        self._retry_after_L = 0
        self._retry_after_R = 0

    def _mark_stale(self, side):
        if side == 'L':
            self._stale_count_L += 1
            if self._stale_count_L > 6:
                self.data_L = -1.0
        else:
            self._stale_count_R += 1
            if self._stale_count_R > 6:
                self.data_R = -1.0

    def _mark_read_failed(self, side, now):
        self._mark_stale(side)
        if side == 'L':
            self._error_count_L += 1
            if self._error_count_L >= 3:
                self._retry_after_L = time.ticks_add(now, 200)
                self._error_count_L = 0
        else:
            self._error_count_R += 1
            if self._error_count_R >= 3:
                self._retry_after_R = time.ticks_add(now, 200)
                self._error_count_R = 0

    def _read_sensor(self, sensor, side, now):
        retry_after = self._retry_after_L if side == 'L' else self._retry_after_R
        if sensor is None or time.ticks_diff(now, retry_after) < 0:
            return
        try:
            if not sensor.data_ready:
                self._mark_stale(side)
                return
            distance, status = sensor.read_measurement()
            sensor.clear_interrupt()
        except (OSError, TimeoutError):
            self._mark_read_failed(side, now)
            return

        distance -= self.bias_L if side == 'L' else self.bias_R
        if distance < 0:
            distance = 0.0
        if side == 'L':
            self.status_L = status
            self._error_count_L = 0
            if status == RANGE_VALID:
                self.data_L = distance
                self._stale_count_L = 0
            else:
                self._mark_stale(side)
        else:
            self.status_R = status
            self._error_count_R = 0
            if status == RANGE_VALID:
                self.data_R = distance
                self._stale_count_R = 0
            else:
                self._mark_stale(side)

    def service(self):
        """Run deferred TOF I2C work from the main loop, never from a timer callback."""
        now = time.ticks_ms()
        if self._desired_side != self._active_side and time.ticks_diff(now, self._sync_retry_ms) >= 0:
            try:
                if self.tof_L and self._active_side == 'L':
                    self.tof_L.stop_ranging()
                if self.tof_R and self._active_side == 'R':
                    self.tof_R.stop_ranging()
                self._active_side = None

                if self._desired_side == 'L' and self.tof_L:
                    self.tof_L.start_ranging(wait=False)
                    self._active_side = 'L'
                elif self._desired_side == 'R' and self.tof_R:
                    self.tof_R.start_ranging(wait=False)
                    self._active_side = 'R'
            except (OSError, TimeoutError):
                self._sync_retry_ms = time.ticks_add(now, 200)
                if self._desired_side in ('L', 'R'):
                    self._mark_read_failed(self._desired_side, now)

        if not self.my_car.if_control_dist or self._active_side is None:
            return
        if time.ticks_diff(now, self._next_read_ms) < 0:
            return
        self._next_read_ms = time.ticks_add(now, self._tof_period_ms)
        self.update_tof(now)

    # 更新tof传感器信息
    def update_tof(self, now=None):
        # 如果没有初始化tof成功则直接退出
        if self.tof_L is None and self.which_one == 'L':
            return
        if self.tof_R is None and self.which_one == 'R':
            return
        if now is None:
            now = time.ticks_ms()
        if self.which_one == 'L':
            self._read_sensor(self.tof_L, 'L', now)
        elif self.which_one == 'R':
            self._read_sensor(self.tof_R, 'R', now)
        else:
            self._read_sensor(self.tof_L, 'L', now)
            self._read_sensor(self.tof_R, 'R', now)

        # 自动选择：优先左传感器，无效则用右传感器
        if not self.which_one:
            if self.data_L != -1.0:
                self.which_one = 'L'
            elif self.data_R != -1.0:
                self.which_one = 'R'

    # 判断数据是否合理
    def is_data_valid(self):
        # 使用滞回判断避免频繁切换：需要连续多次无效才真正失效
        raw_valid = False
        if self.which_one == 'L':
            raw_valid = self.data_L != -1.0 and self.tof_valid_min <= self.data_L <= self.tof_valid_max
        elif self.which_one == 'R':
            raw_valid = self.data_R != -1.0 and self.tof_valid_min <= self.data_R <= self.tof_valid_max

        if raw_valid:
            self._invalid_count = 0
            return True
        else:
            self._invalid_count += 1
            # 连续多次无效才真正失效
            return self.if_init_fil and self._invalid_count <= 4

    # 判断数据是否合理
    def is_data_valid_once(self):
        # 使用滞回判断避免频繁切换：需要连续多次无效才真正失效
        raw_valid = False
        if self.which_one == 'L':
            raw_valid = self.data_L != -1.0 and self.tof_valid_min <= self.data_L <= self.tof_valid_max
        elif self.which_one == 'R':
            raw_valid = self.data_R != -1.0 and self.tof_valid_min <= self.data_R <= self.tof_valid_max
        if raw_valid:
            return True
        else:
            return False
        
    # 重置速度分量
    def reset_speed_weight(self):
        self.my_car.speed_weight = 0.0

    # tof准备
    def ready_tof(self, sensor, fixed_dir, target_obj):
        def choose_sensor(sensor):
            if sensor == 'left':
                self.which_one = 'L'
            elif sensor == 'right':
                self.which_one = 'R'
            else:
                self.which_one = None

        # 选择tof传感器
        choose_sensor(sensor)
        self.dist_pid_L.choose_dist(target_obj)
        self.dist_pid_R.choose_dist(target_obj)
        self.my_car.fixed_direction = fixed_dir
        self.my_car.if_control_dist = True
        self.if_init_fil = False
        self._desired_side = self.which_one
        self._next_read_ms = 0

    # 重置tof传感器信息
    def reset_tof(self):
        self._desired_side = None
        self.data_L, self.data_R = -1.0, -1.0
        self._stale_count_L, self._stale_count_R = 0, 0
        self._invalid_count = 0
        self.which_one = None
        self.status_L, self.status_R = RANGE_VALID, RANGE_VALID
        self.my_car.if_control_dist = False
        self.if_init_fil = False
        self.reset_speed_weight()
        self.my_beep.beep.low()
        
    # 距离控制主函数
    def dist_control(self):
        # 如果没有初始化tof成功则直接退出
        if self.tof_L is None and self.which_one == 'L':
            return
        if self.tof_R is None and self.which_one == 'R':
            return

        # 搬运小熊的时候第一阶段距离更大一些
        if self.which_one == 'L':
            if self.dist_pid_L.current_obj in ['B', 'W'] and len(self.my_plan.path) > 2:
                if self.my_plan.aimed_point_index == 0:
                    self.dist_pid_L.target = self.dist_pid_L.target_B + self.car_dist_increment
                else:
                    self.dist_pid_L.target = self.dist_pid_L.target_B
        elif self.which_one == 'R':
            if self.dist_pid_R.current_obj in ['B', 'W'] and len(self.my_plan.path) > 2:
                if self.my_plan.aimed_point_index == 0:
                    self.dist_pid_R.target = self.dist_pid_R.target_B + self.car_dist_increment
                else:
                    self.dist_pid_R.target = self.dist_pid_R.target_B
        
        # TOF数据由主循环service()更新；定时器中仅使用最近一次缓存。
        if self.is_data_valid():
            self.my_beep.beep.low()
            if self.if_init_fil == False:
                if self.which_one == 'L':
                    self.dist_pid_L.init_filter(self.data_L)
                elif self.which_one == 'R':
                    self.dist_pid_R.init_filter(self.data_R)
                self.if_init_fil = True

            if self.which_one == 'L':
                self.dist_pid_L.compute_pid(self.data_L)
                self.my_car.speed_weight = self.dist_pid_L.pwm_output
            elif self.which_one == 'R':
                self.dist_pid_R.compute_pid(self.data_R)
                self.my_car.speed_weight = self.dist_pid_R.pwm_output
            else:
                # 清零输出
                self.dist_pid_L.reset_pwmout()
                self.dist_pid_R.reset_pwmout()
                self.reset_speed_weight()
        else:
            self.my_beep.beep.high()
            # 清零输出
            self.dist_pid_L.reset_pwmout()
            self.dist_pid_R.reset_pwmout()
            self.reset_speed_weight()
