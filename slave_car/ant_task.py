# 状态机类
from micropython import const
import gc
import math

PI = const(3.1415926)
READY_NAVIGATE = const(0) # 准备导航状态
NAVIGATE = const(1)       # 导航状态
SCAN = const(2)           # 扫描状态
SERVO = const(3)          # 视觉伺服状态
ORBIT = const(4)          # 环绕状态
MOVE = const(5)           # 搬运状态
CALIBRATE = const(6)      # 校准状态
ADJUST = const(7)         # 微调状态
RETURN = const(8)		  # 返回状态
STOP = const(9)           # 停止状态
RETREAT = const(10)       # 后退状态

counter = 0  # 计数器
class TaskController:
    def __init__(self,flash,beep, state, uart, car, path, plan, vision, moving, plan_data, order_manager, art_protocal, slave_protocol, my_tof, angle_pid):
        # 注入对象
        self.my_beep = beep
        self.my_path = path
        self.my_uart = uart
        self.my_plan = plan
        self.my_vision = vision
        self.my_state = state
        self.my_car = car
        self.my_moving = moving
        self.data = plan_data
        self.my_order_manager = order_manager
        self.my_art_protocol = art_protocal
        self.my_slave_protocol = slave_protocol
        self.my_flash_system = flash
        self.my_tof = my_tof
        self.angle_pid = angle_pid
        # 状态映射表：将状态常量映射到对应的处理函数
        self.handlers = {
            READY_NAVIGATE: self.handle_ready_navigate,
            NAVIGATE: self.handle_navigate,
            SCAN:     self.handle_scan,
            SERVO:    self.handle_servo,
            MOVE:     self.handle_move,
            CALIBRATE: self.handle_calibrate,
            ADJUST:   self.handle_adjust,
            RETURN:    self.handle_return,
            STOP:      self.handle_stop,
            RETREAT: self.handle_retreat,
            # ... 其他状态
        }
        self.if_first_run = True
        self.if_blind_box = self.my_flash_system.find_value("IF_BLIND_BOX")
        self.num_clamp_factor = self.my_flash_system.find_value("NUM_CLAMP_FACTOR")
        T_dis = self.my_flash_system.find_value("TENNIS_cla_dis")
        S_dis = self.my_flash_system.find_value("SANDBAG_cla_dis")
        B_dis = self.my_flash_system.find_value("BEAR_cla_dis")
        self.clamp_distance = {'T':T_dis,'S':S_dis,'E':S_dis,'W':B_dis,'B':B_dis}
        self.if_send_return = False
        self.navigate_message = []  # 导航信息：目标点坐标和朝向
        self.pt_buffer = []  # 目标点坐标缓冲区
        self.retreat_message = [] # 后退点坐标缓冲区
        self.current_object = ''  # 当前目标物体种类
        self.last_side = 'D'
        self.blind_buffer = [[0.0, 0.0], 0.0] # 盲盒任务缓冲区
        self.blind_box_state = NAVIGATE # 盲盒状态机状态
        self.car_pos = 'U' # 小车在主车的上侧还是下侧
        self.orbit_state = 0 # 环绕状态机状态

        # 标志位
        self.if_transitioning = True  # 是否正在进行状态转换
        self.if_init_move_data = False # 是否初始化过搬运数据
        self.if_send_to_main = False # 是否向主车发送消息
        self.current_pushed_num = 0
        gc.collect()  # 进行垃圾回收，确保有足够内存用于状态机操作

    # 不同模式下的执行函数
    def run(self):
        if self.if_transitioning:
            self.enter()  # 进入新状态执行一次性的进入函数

        # 获取当前状态对应的函数并执行
        handler = self.handlers.get(self.my_state.state)
        if handler:
            handler()

    # 模式之间的进入和退出函数
    def enter(self):
        state = self.my_state.state
        self.if_transitioning = False  # 进入新状态，重置状态转换标志位

        if state == READY_NAVIGATE:
            # 进入准备导航状态，做好路径规划准备和导航信息准备
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
        elif state == NAVIGATE:
            # 进入导航状态，开始执行路径跟随
            pass
        elif state == SCAN:
            pass
        elif state == SERVO:
            # 进入伺服状态，开始精确对准目标物体
            pass
        elif state == MOVE:
            num_compensation = self.current_pushed_num * self.num_clamp_factor
            self.my_moving.clamp_distance = self.clamp_distance[self.current_object]+num_compensation
            self.my_moving.ready_move(self.pt_buffer[1], self.pt_buffer[0], self.current_object,now_side = self.last_side)
            self.current_pushed_num += 1
            pass
        elif state == CALIBRATE:
            pass
        elif state == ADJUST:
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            # 此时进入盲盒状态
            self.my_vision.if_in_blind = True
            # 发送消息打开摄像头
            self.my_vision.current_servo_object = 'S'
            self.my_order_manager.mode_target()
            self.my_art_protocol.send_object_kind(self.my_vision.current_servo_object)
            self.my_art_protocol.clear_uart_buffer()
            # 判断小车是在上方还是下方
            if self.my_car.y_current < -65.0:
                self.car_pos = 'D'
                self.blind_buffer = [[60.0, -87.0], 57.0]
            else:
                self.car_pos = 'U'
                self.blind_buffer = [[60.0, -43.0], 123.0]
        elif state == RETURN:
            # 进入返回状态，返回起始点或下一任务点
            if len(self.pt_buffer) > 0:
                self.my_path.plan_path(self.pt_buffer[0][0], self.pt_buffer[0][1], ignore_center_rect=True)  # 规划回起始点的路径
                self.my_path.ready_path[-1] = self.pt_buffer[0]
                # 最后插入一个途径点便于计时
                self.my_path.ready_path.insert(-1, [self.pt_buffer[0][0], 10.0])
            else:
                self.my_path.ready_path = [[self.data.fixed_point[3][0], 10.0], self.data.fixed_point[3]]
        elif state == STOP:
            # 进入停止状态，停止所有动作等待下一指令
            self.my_plan.reset_navigate_angle()

    def exit(self):
        global counter
        state = self.my_state.state
        if state == READY_NAVIGATE:
            # 退出准备导航状态，清理路径规划相关资源
            if self.current_object == 'R':
                # 若当前物体信息为回程信息
                self.my_state.state = RETURN  # 直接切换到返回状态
            else:
                self.my_state.state = NAVIGATE  # 直接切换到导航状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == NAVIGATE:
            # 退出导航状态，停止路径跟随
            if self.current_object in ['P', 'M']:
                self.my_plan.reset_navigate_angle()  # 重置导航角度
                self.my_plan.reset_navigate()  # 重置导航标志
                self.my_state.state = READY_NAVIGATE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            elif self.current_object == 'A':
                if self.pt_buffer[1] == 90:
                    if self.my_car.x_current <= self.pt_buffer[0][0]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
                elif self.pt_buffer[1] == -90:
                    if self.my_car.x_current >= self.pt_buffer[0][0]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
                elif self.pt_buffer[1] == 0:
                    if self.my_car.y_current <= self.pt_buffer[0][1]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
                elif self.pt_buffer[1] == 180:
                    if self.my_car.y_current >= self.pt_buffer[0][1]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
                self.my_state.state = CALIBRATE
                self.if_transitioning=True
            elif self.current_object in ['T', 'S', 'E', 'W', 'B']:
                #target_point = self.my_art_protocol.coordinate_receive()
                #if target_point and chr(target_point[2]) == self.current_object:
                # 计数器清零
                self.counter = 0
                self.my_vision.if_send_order = False  # 重置发送指令标志位
                self.my_vision.ready_servo_and_orbit(self.current_object, 'servo')
                self.my_vision.reset_servo_angle()
                self.my_plan.reset_navigate()  # 重置导航相关变量
                self.my_state.state = MOVE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == SCAN:
            pass
        elif state == SERVO:
            # 退出伺服状态，停止精确对准动作
            if self.my_vision.if_finish_servo:
                # 重置环绕角度
                self.my_vision.reset_orbit_angle()
                self.my_vision.if_finish_servo = False  # 重置伺服完成标志
                self.my_state.state = MOVE  # 直接切换到搬运状态
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            elif self.my_plan.if_finish_navigate:
                self.my_vision.if_lost_object = False
                # 将openart置为等待模式
                self.my_order_manager.finish()
                self.my_plan.reset_navigate()
                self.my_slave_protocol.send_slave_state("lost")  # 通知主车丢失物体
                self.my_plan.reset_navigate_angle()
                self.my_state.state = READY_NAVIGATE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == MOVE:
            if self.my_moving.current_state != NAVIGATE:
                self.my_plan.reset_navigate()
                self.my_plan.reset_navigate_angle()
                self.pt_buffer.clear()  # 清空目标点缓冲区
                self.my_state.state = RETURN # 直接切换到返回状态
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            else:
                self.retreat_message = [self.my_car.x_current, self.my_car.y_current]
                self.my_moving.if_finish_move = False  # 重置搬运完成标志
                self.my_plan.reset_navigate_angle()
                self.my_plan.reset_navigate()
                if self.current_object == 'T':self.last_side = 'U'
                elif self.current_object in ['S','E']:self.last_side = 'L'
                else:self.last_side = 'R'
                self.my_state.state = RETREAT  # 直接切换到校准状态
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == CALIBRATE:
            pass
        elif state == ADJUST:
            # 退出调整状态，完成微调后进行必要的状态更新
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_plan.reset_navigate_angle()  # 重置导航角度
            self.my_state.state = STOP  # 直接切换到停止状�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == RETURN:
            # 退出返回状态，完成返回后进行必要的状态更新
            self.my_plan.reset_navigate()  # 重置导航标志
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            if self.if_blind_box:
                self.my_state.state = ADJUST  #进入adjust模式进行盲盒任务
            else:
                self.my_state.state = STOP  # 直接切换到停止状�?
        elif state == STOP:
            # 退出停止状态，准备进入下一任务或待命状态
            self.my_beep.test()  # 任务完成，发出提示音
        elif state == RETREAT:
            # 重置导航标志位
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
    
    def handle_ready_navigate(self):
        global counter
        if self.if_first_run:
            counter += 1
            if counter <= 50:
                return
        # 进入准备导航状态，做好路径规划准备和导航信息准备
        path = self.my_slave_protocol.get_path_list()  # 从从车协议中获取路径信息
        if path:
            # 只有当路径信息为过渡或者回城时才记录目标点坐标
            horizon_stop_threshold = 10
            if path[0] not in ['P']:
                self.pt_buffer = [path[2], path[1]]  # 储存目标坐标
                self.navigate_message = []  # 收到物体坐标先不导航
            else:
                if not self.if_init_move_data:
                    if abs(path[1] - 180) < 1e-3:
                        self.my_moving.next_postion = 'r'
                        self.last_side = 'U'
                    elif abs(path[1] - 90) < 1e-3:
                        self.my_moving.next_postion = 'r'
                        self.last_side = 'L'
                    elif abs(path[1] - (-90)) < 1e-3:
                        self.my_moving.next_postion = 'l'
                        self.last_side = 'R'
                    else:
                        self.last_side = 'D'
                        self.my_moving.next_postion = 'l'

                    self.if_init_move_data = True

                # 进行路径规划
                tx=path[2][0]
                ty=path[2][1]
                if abs(tx+1)<1e-3 and abs(ty+1)<1e-3:
                    self.navigate_message = [[-1,-1], path[1]]  # 只保留角度
                else:
                    #规划停在主车左/右侧
                    current_yaw_deg = self.my_car.now_yaw * 180.0 / PI
                    if current_yaw_deg > -45.0 and current_yaw_deg <= 45.0: 
                        current_turn_deg = 0.0
                    elif current_yaw_deg > 45.0 and current_yaw_deg <= 135.0:
                        current_turn_deg = 90.0
                    elif current_yaw_deg > 135.0 or current_yaw_deg <= -135.0:
                        current_turn_deg = 180.0
                    elif current_yaw_deg > -135.0 and current_yaw_deg <= -45.0:
                        current_turn_deg = -90.0
                    if self.if_first_run:
                        horizon_stop_threshold = 20
                    if current_turn_deg == 0 or current_turn_deg == 180:
                        tx=min(max(path[2][0]-horizon_stop_threshold,self.my_car.x_current),path[2][0]+horizon_stop_threshold)
                    else:
                        ty=min(max(path[2][1]-horizon_stop_threshold,self.my_car.y_current),path[2][1]+horizon_stop_threshold)
                    self.my_path.plan_path(tx, ty)  # 传入目标坐标进行路径规划
                    pathh = self.my_path.ready_path  # 获取规划好的路径
                    if self.if_first_run:
                        counter = 0
                        self.if_first_run = False
                        pathh.insert(0, [self.my_car.x_current, self.my_car.y_current+50.0])
                    self.navigate_message = [pathh, path[1]]  # 目标坐标和转向角度
            self.current_object = path[0]  # 当前物体种类
            self.my_plan.current_object = self.current_object  # 将当前物体种类传递给路径跟随模块
            # 测试
            # self.my_uart.write(f"Ready to navigate to {self.current_object} at {self.navigate_message[0]} with turn {self.navigate_message[1]}\r\n")  # 调试信息
            # self.my_uart.write(f"{self.my_path.ready_path}\r\n")
            self.exit()  # 退出当前状态，进入导航状态
    def handle_navigate(self):
        # if state == NAVIGATE
        if self.navigate_message:
            if self.navigate_message[0][0] == -1 and self.navigate_message[0][1] == -1:
                self.my_plan.navigate(target_turn_angle = self.navigate_message[1])
            else:
                self.my_plan.navigate(path = self.navigate_message[0], target_turn_angle = self.navigate_message[1])
        else:
            self.my_plan.if_finish_navigate=True#直接退出
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入扫描状态
    
    def handle_scan(self):
        # if state == SCAN
        pass

    def handle_servo(self):
        # if state == SERVO
        if self.my_vision.if_lost_object == False:
            self.my_vision.visual_servo_control()
        else:
            # 若丢失物体则四处移动寻找物体
            x = self.my_car.x_current
            y = self.my_car.y_current
            now_yaw = self.my_car.now_yaw  # 弧度，0=北(+Y)，90°=东(+X)
            # 车身右方(+X): (cos(now_yaw), -sin(now_yaw))
            # 车身左方(-X): (-cos(now_yaw), sin(now_yaw))
            right_x = x + 15.0 * math.cos(now_yaw)
            right_y = y - 15.0 * math.sin(now_yaw)
            left_x = x - 15.0 * math.cos(now_yaw)
            left_y = y + 15.0 * math.sin(now_yaw)
            self.my_plan.navigate(path = [[right_x, right_y], [left_x, left_y], self.pt_buffer[0]], target_turn_angle = self.pt_buffer[1])
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.current_object:
                self.my_vision.if_send_order = False
                self.my_vision.ready_servo_and_orbit(chr(target_point[2]), 'servo', target_point)
                self.my_plan.reset_navigate()
                self.my_vision.if_lost_object = False
                self.my_order_manager.mode_target() # 打开目标识别模式
        if self.my_vision.if_finish_servo or self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入搬运状态

    def handle_move(self):
        # if state == MOVE
        self.my_moving.moving()
        if self.my_moving.if_finish_move:
            self.exit()  # 退出当前状态，进入下一个状态

    def handle_retreat(self):
        # if state == ADJUST
        self.my_plan.navigate(path = [self.retreat_message])
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入下一个状态
    
    def handle_calibrate(self):
        pass
    
    def handle_adjust(self):
        if self.blind_box_state == NAVIGATE:
            self.my_plan.navigate(path = [self.blind_buffer[0]], target_turn_angle = self.blind_buffer[1])

            target_point = self.my_art_protocol.coordinate_receive()
            if self.angle_pid.if_finish_turn() and target_point and chr(target_point[2]) == self.my_vision.current_servo_object:
                self.my_vision.ready_servo_and_orbit(chr(target_point[2]), 'servo', target_point)
                self.my_vision.object_radius = 19.0
                self.my_vision.reset_servo_angle()
                self.my_plan.reset_navigate()
                self.my_vision.if_finish_servo = False
                self.blind_box_state = SERVO
        elif self.blind_box_state == SERVO:
            self.my_vision.visual_servo_control()

            if self.my_vision.if_finish_servo:
                if not self.if_send_to_main:
                    self.my_slave_protocol.send_slave_state("get")
                    self.if_send_to_main = True

                if self.my_slave_protocol.get_start_signal():
                    self.blind_box_state = ORBIT
                    self.my_vision.reset_orbit()
                    self.my_vision.reset_orbit_angle()
        elif self.blind_box_state == ORBIT:
            if self.orbit_state == 0:
                if self.car_pos == 'U':
                    self.my_vision.orbit_control(-120.0)
                else:
                    self.my_vision.orbit_control(180.0)

                if self.my_vision.if_finish_orbit:
                    self.orbit_state = 1
                    self.my_vision.reset_orbit()
                    self.my_vision.reset_orbit_angle()
            elif self.orbit_state == 1:
                if self.car_pos == 'U':
                    self.my_vision.orbit_control(0.0)
                else:
                    self.my_vision.orbit_control(-60.0)

                if self.my_vision.if_finish_orbit:
                    self.orbit_state = 2
                    self.my_vision.reset_orbit()
                    self.my_vision.reset_orbit_angle()
            elif self.orbit_state == 2:
                if self.car_pos == 'U':
                    self.my_vision.orbit_control(120.0)
                else:
                    self.my_vision.orbit_control(60.0)

                if self.my_vision.if_finish_orbit:
                    self.orbit_state = 3
                    self.my_vision.reset_orbit()
                    self.my_vision.reset_orbit_angle()
                    self.exit()

    def handle_return(self):
        # if state == RETURN
        self.my_plan.navigate(path = self.my_path.ready_path)  # 返回起始点
        if self.my_plan.finished_dist >= 15 and not self.if_send_return:
            self.my_slave_protocol.send_slave_state("lost")
            self.if_send_return = True
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入停止状态

    def handle_stop(self):
        # if state == STOP
        self.my_plan.stop()
