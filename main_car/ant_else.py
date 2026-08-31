from micropython import const
import time
import gc

PI = const(3.1415926)
READY_NAVIGATE = const(0)   # 准备导航状�?
NAVIGATE = const(1)       # 导航状�?
SCAN = const(2)           # 扫描状�?
SERVO = const(3)          # 视觉伺服状�?
ORBIT = const(4)          # 环绕状�?
MOVE = const(5)           # 搬运状�?
CALIBRATE = const(6)      # 校准状�?
ADJUST = const(7)           # 微调状�?
RETURN = const(8)		    # 返回状�?
STOP = const(9)           # 停止状�?
RETREAT = const(10)

# 计数�?
counter = 0 
##############################【蜂鸣器�?#############################
BEEP_OFF = const(0)
BEEP_ON = const(1)
##############################【flash系统操作�?#############################
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
        # 尝试解析为整�?只支持十进制)
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
    
    # 打开参数文件并进行解析，传入一个文件路径，返回一个字�?
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
            # 解析变量�?
            # 如果值以双引号开头和结尾，认为它是一个列表的字符串表示，替换为方括号后使用eval解析为列�?
            if var_value[0] == '"' and var_value[-1] == '"':  # 列表类型
                if var_name == "rogue_planning":
                    try:
                        self.config[var_name] = eval("[" + var_value[1:-1] + "]")
                    except Exception as e:
                        print(f"Error: Failed to evaluate {var_name} = {var_value}")
                        self.beep.beep_warn()
                else:
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
            elif var_value[0] == "'" and var_value[-1] == "'":
                # 单引号包裹的值视为字符串（如 'L' → "L"）
                self.config[var_name] = var_value[1:-1]
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

        # 检�?rogue_planning: [[(float, float), str, str, [(str, float), ...]]]
        if "rogue_planning" in self.config:
            rp = self.config["rogue_planning"]
            if type(rp) is not list:
                print("Error: 'rogue_planning' 必须是列?")
                error_flag = True
            else:
                for plan in rp:
                    if type(plan) is not list or len(plan) != 4:
                        print("Error: 'rogue_planning' 中的元素必须是长度为4的列�?")
                        error_flag = True
                        break
                    p_coord, p_kind, p_dir, p_cond = plan
                    if type(p_coord) is not tuple or len(p_coord) != 2 or not all(type(x) in (int, float) for x in p_coord):
                        print("Error: 'rogue_planning' 的坐标格式错误，应为 (float, float)")
                        error_flag = True
                    if type(p_kind) is not str or p_kind not in ('E', 'S', 'B', 'W', 'T'):
                        print("Error: 'rogue_planning' 的物体种类格式错误，应为字符�?(�?'P', 'E', 'S', 'B', 'W', 'T')")
                        error_flag = True
                    if type(p_dir) is not str or p_dir not in ('U', 'D', 'L', 'R'):
                        print("Error: 'rogue_planning' 的方向格式错误，应为字符�?(�?'U', 'D', 'L', 'R')")
                        error_flag = True
                    if type(p_cond) is not list:
                        print("Error: 'rogue_planning' 的条件格式错误，应为列表")
                        error_flag = True
                    else:
                        for cond in p_cond:
                            if type(cond) is not tuple or len(cond) != 2 or type(cond[0]) is not str or type(cond[1]) not in (int, float):
                                print("Error: 'rogue_planning' 中条件元素格式错误，应为 (str, float)")
                                error_flag = True
                                break

        # 检�?cube_obstacles: [(float, float, float, float), ...]
        if "cube_obstacles" in self.config:
            co = self.config["cube_obstacles"]
            if type(co) is not list:
                print("Error: 'cube_obstacles' 必须是列�?")
                error_flag = True
            else:
                for obs in co:
                    if type(obs) is not tuple or len(obs) != 4 or not all(type(x) in (int, float) for x in obs):
                        print("Error: 'cube_obstacles' 的元素格式错误，应为包干4个数字的元组 (float, float, float, float)")
                        error_flag = True
                        break

        # 检�?circle: [(float, float), ...]
        if "circle" in self.config:
            cr = self.config["circle"]
            if type(cr) is not list:
                print("Error: circle must be list")
                error_flag = True
            else:
                for cir in cr:
                    if type(cir) is not tuple or len(cir) != 2 or not all(type(x) in (int, float) for x in cir):
                        print("Error: 'circle' 的元素格式错误，应为包干2个数字的元组 (float, float)")
                        error_flag = True
                        break

        if error_flag:
            self.beep.beep_warn()

# 日志系统
class write_system:
    def __init__(self, flash_sys: flash_system, beep, file_path: str):
        self.flash_sys = flash_sys
        # 注入蜂鸣器对象，用于警报
        self.beep = beep
        self.num = 1
        # 传入文件路径
        self.file_path = file_path  # type: str
        # 环形字节缓冲区：一次性预分配固定内存，写入过程零动态分配，避免内存碎片
        self.buf_size = 1024        # 缓冲区字节上限（可调）
        self.buf = bytearray(self.buf_size)
        self.if_write_log = self.flash_sys.find_value("if_write_log")
        self.head = 0               # 已写入的有效字节数（数据始终从 buf[0] 连续存放）
        gc.collect()

    def write_str(self, line: str):
        # 如果未开启日志则直接返回
        if not self.if_write_log:
            return

        print(line)

        '''
        """将一行日志写入缓冲区；空间不足时丢弃最旧的一行（循环队列语义）"""
        if not line.endswith("\n"):
            line += '\n'
        try:
            data = line.encode('utf-8')
        except Exception:
            return
        n = len(data)

        # 单行超过整个缓冲区：只保留末尾部分
        if n > self.buf_size:
            data = data[n - self.buf_size:]
            n = self.buf_size

        # 空间不足时，丢弃最旧的一行（从开头找第一个换行符）
        while self.head + n > self.buf_size:
            drop = 1
            while drop < self.head and self.buf[drop - 1] != 10:  # 10 == ord('\n')
                drop += 1
            # 前移剩余数据，腾出 drop 字节空间
            for i in range(drop, self.head):
                self.buf[i - drop] = self.buf[i]
            self.head -= drop

        # 追加新数据
        for i in range(n):
            self.buf[self.head + i] = data[i]
        self.head += n
       '''

    def write_in(self) -> None:
        # 如果未开启日志则直接返回
        if not self.if_write_log:
            return
        '''
        """将缓冲区中的所有内容一次性刷入文件"""
        if self.head == 0:
            return
        try:
            with open(self.file_path, 'ab') as f:
                f.write(self.buf[0:self.head])
        except Exception as e:
            # 写入失败，缓冲区数据原封不动，下次 write_in 会重试
            print(f"Error: Failed to write to {self.file_path}: {e}")
            return
        self.head = 0
        '''
    def init_write(self) -> None:
        self.num = 1
        try:
            with open(self.file_path, 'r') as f:
                first_line = f.readline()
            prefix = "This is the "
            suffix = "th log of the main car."
            if first_line.startswith(prefix):
                end = first_line.find(suffix, len(prefix))
                if end >= 0:
                    self.num = int(first_line[len(prefix):end]) + 1
        except Exception:
            # The first boot may not have created a log file yet.
            self.num = 1
        try:
            with open(self.file_path, 'w') as f:
                f.write(f"This is the {self.num}th log of the main car.\n")
        except Exception as e:
            print(f"Error: Failed to write to {self.file_path}: {e}")

class beep:
    __slots__ = ('beep', 'beep_state')
    def __init__(self, beep):
        # 注入蜂鸣器对象，用于警报
        self.beep = beep
        self.beep_state = BEEP_OFF
    # 蜂鸣器警告函�?�?声，�?00ms响一声，每次持续50ms)
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

    # 蜂鸣器测试函�?响一声，持续50ms)
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
        

##############################【uart串口解析数据�?#############################
# 指令管理�?
class order_manager:
    def __init__(self, flash_sys :flash_system, uart):
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

    def trans_to_mode_detect(self):
        self.my_uart.write("m")

    # 切换到apriltag识别模式
    def mode_apriltag(self):
        self.my_uart.write("R")

    def send_object_kind(self, object_kind):
        self.my_uart.write(object_kind.lower())

    def clear_knock(self):
        self.my_uart.write("c")

    # 切换到左右边界识别模�?
    def mode_boundary_lf(self):
        self.my_uart.write("L")

    # 色块识别模式
    def color_target(self):
        self.my_uart.write("C")

    def mode_detect(self):
        self.my_uart.write("A")

    # 当前模式结束
    def finish(self):
        self.my_uart.write("F")    

        
# 状态机解析串口数据�?
class UARTProtocol:
    def __init__(self, uart):
        # 注入串口对象
        self.my_uart = uart
        self.state_coordinate = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待x, 3:等待y, 4:等待物体种类, 5:等待帧尾
        self.state_apriltag = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待x, 3:等待y, 4:等待角度, 5:等待帧尾
        self.coordinate_buffer = [0, 0, 0, 0, '', 0]
        self.apriltag_buffer = [0, 0, 0, 0, 0, 0, 0]
        self.byte_count = 0
        self.state_detect_all_objects = 0 # 0:等待帧头1, 1:等待物体数量, 2:等待发送物体讯�? 5:等待帧尾
        self.detect_buffer = [0,[]]
        self.object_buffer = ['',0,0]
        self.state_object = 0 # 0:等待x, 1:等待y, 2:等待物体种类
        gc.collect()
    def clear_uart_buffer(self):
        self.state_coordinate = 0
        self.state_apriltag = 0
        self.state_detect_all_objects = 0
        self.state_object = 0
        self.detect_buffer = [0,[]]
        self.coordinate_buffer = [0, 0, 0, 0, '', 0]
        self.object_buffer = ['',0,0]
        self.my_uart.read(self.my_uart.any())#清空缓冲�?
    # 非阻塞接收并解析物体中心的像素点坐标  
    def coordinate_receive(self):
        last_valid_frame = None
        # 持续读取直到处理完当前缓冲区的所有数�?
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
                    # 解析成功，保存当前帧，但【不要】清空缓冲区，【不要】立即返�?
                    x, y = self.coordinate_buffer[2], self.coordinate_buffer[3]
                    last_valid_frame = (x, y, self.coordinate_buffer[4])
                    self.state_coordinate = 0 
                else:
                    self.state_coordinate = 0
                
        # 循环结束后，返回缓冲区里最新的一�?
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
                    self.object_buffer[1] = byte
                    self.state_object = 1
                elif self.state_object == 1:
                    self.object_buffer[2] = byte
                    self.state_object = 2
                elif self.state_object == 2:
                    if byte in [84,69,83,66,87]:
                        self.object_buffer[0] = byte
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
    # 发送物体种�?
    def send_object_kind(self, object_kind):
        self.my_uart.write(object_kind.lower())
    
    # 非阻塞接收并解析apriltag码的像素点坐标和角度  
    def apriltag_receive(self):
        last_valid_frame = None
        # 持续读取直到处理完当前缓冲区的所有数�?
        while self.my_uart.any():	
            byte = self.my_uart.read(1)[0]
            
            if self.state_apriltag == 0:
                if byte == 0xA5:
                    self.state_apriltag = 1
            elif self.state_apriltag == 1:
                if byte == 0xA8:
                    self.state_apriltag = 2
                else:
                    self.state_apriltag = 0
            elif self.state_apriltag == 2:
                self.apriltag_buffer[2] = byte
                self.state_apriltag = 3
            elif self.state_apriltag == 3:
                self.apriltag_buffer[3] = byte
                self.state_apriltag = 4
            elif self.state_apriltag == 4:
                self.apriltag_buffer[4] = byte
                self.state_apriltag = 5
            elif self.state_apriltag == 5:
                self.apriltag_buffer[5] = byte
                self.state_apriltag = 6
            elif self.state_apriltag == 6:
                if byte == 0x5B:
                    # 解析成功，保存当前帧，但【不要】清空缓冲区，【不要】立即返�?
                    last_valid_frame = [self.apriltag_buffer[2], self.apriltag_buffer[3], ((self.apriltag_buffer[5] << 8 | self.apriltag_buffer[4]) / 10 - 90)]
                    self.state_apriltag = 0 
                else:
                    self.state_apriltag = 0
                
        # 循环结束后，返回缓冲区里最新的一�?
        return last_valid_frame

# 主从机通信�?
class LinkProtocol:
    def __init__(self, uart3):
        # 注入串口对象
        self.my_uart3 = uart3
        # 创建字节流缓冲区
        self.raw_buffer = b''           
        self.max_buf = 128         # 缓冲区最大长度，防止内存泄漏
        self.start_idx = 0          # 上次成功解析后剩余数据的起始索引（相对于raw_buffer�?
        self.end_idx = 0            # 上次成功解析后剩余数据的结束索引（相对于raw_buffer�?

        gc.collect()

    # 用于主车向从车发送目标物体种类及规划好的路径坐标�?
    def send_path(self, target_object, target_turn, target_point):
        """
        发送路径点列表 (非阻�?
        格式: #P/S/B/T/E/W/A,0.0,120.5,80.1!
        :param target_object: 目标物体种类
        :param target_turn: 目标转向角度
        :param target_point: (x, y) 目标点坐�?
        """
        if isinstance(target_object, int):
            target_object = chr(target_object)
        packet = "#" + target_object + "," + str(target_turn) + "," + "{:.1f},{:.1f}".format(*target_point) + "!"
        self.my_uart3.write(packet.encode('utf-8'))

    # 向从车发送开始信�?
    def send_start(self):
        self.my_uart3.write('S'.encode('utf-8'))
        
    def get_slave_state(self):
        """
        解析从车状态包 (非阻�?
        包格�? 'R' (ready), 'L' (lost), 'F' (finish), 'G' (get) 等单字节状态指�?
        :return: 'ready', 'lost', 'finish', 'get' �?None
        """
        if self.my_uart3.any():
            try:
                byte = self.my_uart3.read(1)[0]
                if byte == ord('R'):
                    return "ready"
                elif byte == ord('L'):
                    return "lost"
                elif byte == ord('F'):
                    return "finish"
                elif byte == ord('G'):
                    return "get"
                else:
                    return None
            except:
                return None
        else:
            return None