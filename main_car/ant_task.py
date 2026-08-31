from micropython import const
import gc,time
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
counter = 0 
class TaskController:
    def __init__(self,my_write_system,flash_system,object_plan, beep, state, uart, car, path, plan, vision, moving, plan_data, order_manager, art_protocal, main_protocol,uart_debug,angle_pid):
        # 注入对象
        self.my_write_system = my_write_system
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
        self.my_main_protocol = main_protocol
        self.object_plan = object_plan
        self.uart_debug = uart_debug
        self.my_flash_system = flash_system
        self.angle_pid = angle_pid
        # 状态映射表：将状态常量映射到对应的处理函�?
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
            # ... 其他状�?
        }
        self.scan_empty_counter = 0
        self.first_return = True
        self.if_send_return_message = False
        self.if_rogue_plan=self.data.if_rogue_plan
        self.navigate_message = []  # 导航信息：目标点坐标和朝�?
        self.slave_navigate_message = []  # 从车导航信息：目标点坐标和朝�?
        self.current_object = ''  # 当前目标物体种类
        self.now_objects = []
        # 标志�?
        self.if_start_task = False  # 是否开始任务
        self.if_transitioning = True  # 是否正在进行状态转�?
        self.if_send_path = False  # 是否已经发送路径规划信�?
        self.detected_num = 0
        self.if_send_detect_message = False
        self.if_model_detect = self.my_flash_system.find_value("if_model_detect")
        self.SUDOKU_length_x = self.my_flash_system.find_value("SUDOKU_length_x")
        self.SUDOKU_width_y = self.my_flash_system.find_value("SUDOKU_width_y")
        self.use_scan_point = self.my_flash_system.find_value("USE_SCAN_POINT")
        self.if_back = self.my_flash_system.find_value("IF_BACK")
        self.if_blind_box = self.my_flash_system.find_value("IF_BLIND_BOX")
        self.scan_num = self.my_flash_system.find_value("scan_num")
        self.scan_waiting_count = 0
        self.planned_scan_path = []
        self.dangerous_object_kinds = set()
        self.max_num_T = self.my_flash_system.find_value("max_num_T")
        self.max_num_S = self.my_flash_system.find_value("max_num_S")
        self.max_num_E = self.my_flash_system.find_value("max_num_E")
        self.max_num_B = self.my_flash_system.find_value("max_num_B")
        self.max_num_W = self.my_flash_system.find_value("max_num_W")
        self.retreat_lenth = self.my_flash_system.find_value("retreat_lenth")
        self.obj_num_ = {'T':0,'S':0,'B':0,'W':0,'E':0}
        self.angle_buffer = 0.0
        self.max_pos = 0.0
        self.if_plan_scan =False#是否规划出扫描路径
        self.if_end_first_scan = False#是否完成第一次扫描，全局只扫一次
        self.if_first_round = True#是否是第一轮用于判断是否需插入从边线返回途经点
        self.if_choose_object = False#用于判断readynavigate是否成功选择到物体并readymove
        self.black_state = READY_NAVIGATE
        self.black_angle = 0
        self.scan_side = self.my_flash_system.find_value("scan_side")  # 在哪边进行扫描
        if self.scan_side not in ['D', 'L', 'R', 'U']:
            # print("Write invalid scan_side in flash, default to 'D'")
            self.my_beep.beep_warn()    # 蜂鸣器进行提醒此时扫描边参数输入错误
            self.scan_side = 'D'  # 默认扫描在下边

        if self.use_scan_point == 4:
            if self.scan_side in ['D', 'U']:
                self.last_side = 'U'
                self.my_moving.next_postion = 'l'
            else:
                self.last_side = 'R'
                self.my_moving.next_postion = 'r'
        else:
            self.last_side = self.scan_side  # 开始边与扫描边一致

            if self.scan_side in ['D', 'R']:
                self.my_moving.next_postion = 'r'
            else:
                self.my_moving.next_postion = 'l'

        if self.use_scan_point == 1:
            self.fixed_scan_point = [[self.data.center_rect[0], 45]] # type: ignore # type: list
        elif self.use_scan_point == 4:
            if self.scan_side == 'D' or self.scan_side == 'U':
                self.fixed_scan_point = [[[self.data.center_x - self.data.lenth, self.data.fixed_point[1][1]], 0], [[self.data.center_x + self.data.lenth*0.5, self.data.fixed_point[1][1]], 0], [[self.data.center_x + self.data.lenth, self.data.fixed_point[2][1]], 180], [[self.data.center_x - self.data.lenth * 0.5, self.data.fixed_point[2][1]], 180]]
            else:
                self.fixed_scan_point = [[[self.data.fixed_point[1][0], self.data.center_y - self.data.lenth], 90], [[self.data.fixed_point[1][0], self.data.center_y+self.data.lenth*0.5], 90], [[self.data.fixed_point[2][0], self.data.center_y + self.data.lenth], -90], [[self.data.fixed_point[2][0], self.data.center_y-self.data.lenth*0.5],- 90]]
        else:
            # self.use_scan_point == 2
            if self.scan_side == 'D':
                self.fixed_scan_point = [[[self.data.center_x - self.data.lenth, self.data.fixed_point[1][1]], 0], [[self.data.center_x + self.data.lenth*0.5, self.data.fixed_point[1][1]], 0]] # type: ignore
            elif self.scan_side == 'L':
                self.fixed_scan_point = [[[self.data.fixed_point[1][0], self.data.center_y - self.data.lenth], 90], [[self.data.fixed_point[1][0], self.data.center_y+self.data.lenth*0.5], 90]] # type: ignore 
            elif self.scan_side == 'R':
                self.fixed_scan_point = [[[self.data.fixed_point[2][0], self.data.center_y - self.data.lenth], -90], [[self.data.fixed_point[2][0], self.data.center_y+self.data.lenth*0.5],- 90]] # type: ignore
            elif self.scan_side == 'U':
                self.fixed_scan_point = [[[self.data.center_x - self.data.lenth*0.5, self.data.fixed_point[2][1]], 180], [[self.data.center_x + self.data.lenth, self.data.fixed_point[2][1]], 180]] # type: ignore
        gc.collect()  # 进行垃圾回收，确保有足够内存用于状态机操作
    def if_danger(self,sp,num):
        if sp == 'T':return num>self.max_num_T
        elif sp == 'S':return num>self.max_num_S
        elif sp == 'E':return num>self.max_num_E
        elif sp == 'W':return num>self.max_num_W
        elif sp == 'B':return num>self.max_num_B
        return True
    
    # 不同模式下的执行函数
    def run(self):
        # 开始任务标志
        if not self.if_start_task:
            self.if_start_task = True
            
        if self.if_transitioning:
            self.enter()  # 进入新状态执行一次性的进入函数
        # 获取当前状态对应的函数并执�?
        handler = self.handlers.get(self.my_state.state)
        if handler:
            handler()

    # 模式之间的进入和退出函�?
    def enter(self):
        state = self.my_state.state
        self.if_transitioning = False  # 进入新状态，重置状态转换标志位
        if state == READY_NAVIGATE:
            self.my_plan.reset_navigate_angle()
            self.object_plan.reset_judge()
            self.if_choose_object = False
            # 进入准备导航状态，做好路径规划准备和导航信息准�?
            pass
        elif state == NAVIGATE:
            # 进入导航状态，开始执行路径跟�?
            pass
        elif state == SCAN:
            # 进入扫描状态，开始寻找目标物�?
            self.detected_num = 0
            self.my_plan.reset_navigate()
            self.if_send_detect_message = False
            #self.my_order_manager.mode_detect()
            self.object_plan.reset_judge()
            if self.if_rogue_plan:
                self.my_art_protocol.send_object_kind(self.current_object)  # 发送目标物体种类信�?
            #self.scan_message.append([self.my_car.x_current, self.my_car.y_current])  # 记录扫描状态开始时小车的位置，作为后续判断是否迷路的参�?
        elif state == SERVO:
            # 进入伺服状态，开始精确对准目标物�?
            pass
        elif state == MOVE:
            # 进入搬运状态，开始搬运物�?
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            self.my_moving.my_photo.reset_photo()
            pass
        elif state == CALIBRATE:
            pass
        elif state == ADJUST:
            self.black_state = READY_NAVIGATE
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
        elif state == RETURN:
            # 进入返回状态，返回起始点或下一任务�?
            p1 = [min(max(20,self.my_car.x_current),self.data.FIELD_W-15),min(max(15,self.my_car.y_current),self.data.FIELD_H-15)]
            # 最后插入一个途径点便于计�?
            if self.first_return:
                self.my_path.plan_path(self.data.fixed_point[3][0], self.data.fixed_point[3][1], ignore_center_rect=True)  # 规划回起始点的路�?
                self.my_path.ready_path[-1] = self.data.fixed_point[3]
                self.my_path.ready_path.insert(-1, [self.data.fixed_point[3][0], 10.0])
            else:
                self.my_path.plan_path(self.data.fixed_point[4][0], self.data.fixed_point[4][1], ignore_center_rect=True)  # 规划回起始点的路�?
                self.my_path.ready_path[-1] = self.data.fixed_point[4]
                self.my_path.ready_path.insert(-1, [self.data.fixed_point[4][0], 10.0])
            self.my_path.ready_path.insert(0, p1)
            # self.my_uart.write(f"Path: {self.my_path.ready_path}")  # 测试：打印路径点
        elif state == STOP:
            # 进入停止状态，停止所有动作等待下一指令
            self.my_plan.reset_navigate_angle()
        elif state == RETREAT:
            pass

    def exit(self):
        state = self.my_state.state
        if state == READY_NAVIGATE:
            # 退出准备导航状态，清理路径规划相关资源
            self.if_send_path = False  # 重置路径发送标志位
            self.my_plan.reset_navigate()  # 重置导航标志
            if not self.if_end_first_scan:
                self.my_state.state = SCAN
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
                return
            
            if not self.if_choose_object:
                self.my_plan.reset_navigate()
                self.my_state.state = RETURN
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
                return
            
            self.my_state.state = MOVE  # 直接切换到导航状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == NAVIGATE:
            pass
        elif state == SCAN:
            self.planned_scan_path.clear()
            self.if_plan_scan = False
            # 退出扫描状态，停止寻找目标物体
            if not self.if_end_first_scan:
                self.my_plan.reset_navigate()
                self.my_plan.reset_navigate_angle()
                self.my_state.state = RETURN
                self.if_transitioning = True  # 退出当前状态，直接回家
                return
            else:
                self.my_plan.reset_navigate()
                self.my_plan.reset_navigate_angle()
                self.my_state.state = READY_NAVIGATE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == SERVO:
            pass
        elif state == MOVE:
            if self.current_object == 'T':self.last_side = 'U'
            elif self.current_object == 'S' or self.current_object == 'E':self.last_side = 'L'
            elif self.current_object == 'W' or self.current_object == 'B':self.last_side = 'R'
            else:
                self.my_plan.reset_naviself.if_finish_movegate_angle()
                self.my_state.state = RETURN 
            # print(f"{self.last_side}, {self.my_moving.next_postion}, {self.first_return}")
            if self.first_return:
                if self.data.current_index >= self.data.total_objects_num - 1 or not self.now_objects or self.my_moving.current_state != NAVIGATE:
                    if self.last_side == 'L' and self.my_moving.next_postion == 'l':self.first_return = False
                    elif self.last_side == 'R' and self.my_moving.next_postion == 'r':self.first_return = False
                    elif self.last_side == 'U' and self.my_moving.next_postion == 'l':self.first_return = False
                    elif self.last_side == 'D' and self.my_moving.next_postion == 'r':self.first_return = False
                    if self.first_return:
                        self.my_plan.reset_navigate_angle()
                        self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
                        self.my_moving.reset_move()  # 重置搬运标志
                        self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
                    return
                else:
                    self.if_send_path = False
                    self.data.current_index += 1
                    self.my_plan.reset_navigate()
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = READY_NAVIGATE
                    # 测试光电管矫正效果
                    self.my_moving.reset_move()  # 重置搬运标志
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
            else:
                if not self.if_send_return_message:
                    self.my_main_protocol.send_path('R', 999, self.data.fixed_point[3])  # 发送路径信息给从车
                    self.if_send_return_message = True
                if self.my_main_protocol.get_slave_state() == 'lost':
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
                    self.my_moving.reset_move()  # 重置搬运标志
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
                return
        elif state == CALIBRATE:
            pass
        elif state == ADJUST:
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_plan.reset_navigate_angle()
            self.my_state.state = STOP  # 直接切换到停止状�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
        elif state == RETURN:
            if not self.if_send_path:
                # 发送路径信息给从车
                self.my_main_protocol.send_path('R', 999, self.data.fixed_point[4]) 
            # 退出返回状态，完成返回后进行必要的状态更�?
            self.if_send_path = True
            self.my_plan.reset_navigate()  # 重置导航标志
            if self.if_blind_box:
                self.my_state.state = ADJUST  #进入adjust模式进行盲盒任务
            else:
                self.my_state.state = STOP  # 直接切换到停止状�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
        elif state == STOP:
            # 退出停止状态，准备进入下一任务或待命状�?
            self.my_beep.test()  # 任务完成，发出提示音
        elif state == RETREAT:
            pass
    def handle_ready_navigate(self):
        pass
    def handle_navigate(self):
        pass
    # 处理物体信息（将像素坐标转换为世界坐标）
    def handle_object_info(self, ob_info, angle):
        """将单帧物体列表的像素坐标转换为世界坐标，返回新列表"""
        real_ob_info = []
        for ob in ob_info[1]:
            sp, x, y  = ob
            if x<15 or y<10 or x>145 or y >110:
                continue
            kind = chr(sp)
            # 更新当前物体种类，便于选择物体高度
            self.my_vision.current_servo_object = kind
            if self.use_scan_point > 2:  
                if angle == 180 or angle == -90: 
                    limit_y = 40
                else: 
                    limit_y = 68
            else: 
                limit_y = None

            real_point = self.my_vision.predict_point(x, y, limit_y = limit_y)
            if not real_point: continue
            if not self.my_vision.if_in_rect(real_point[0],real_point[1]): continue
            real_ob_info.append((kind,real_point[0], real_point[1]))

        self.my_vision.current_servo_object = ''  # 重置当前物体种类
        return real_ob_info
    
    def snap_objects_to_nine_grid(self, objects, cell_x, cell_y, maxNum,
                                  grid_center_x=160.0, grid_center_y=120.0):
        """Snap detections into the fixed 3x3 grid with bounded allocations."""
        if not objects:
            return []
        kind_grid = self.object_plan.nine_grid
        slots = [[-1, -1, -1], [-1, -1, -1], [-1, -1, -1]]
        for i in range(3):
            kind_grid[i][0] = ''
            kind_grid[i][1] = ''
            kind_grid[i][2] = ''
        half_x = cell_x * 0.5
        half_y = cell_y * 0.5
        for obj_idx in range(len(objects)):
            try:
                kind, x, y = objects[obj_idx]
                if kind not in self.obj_num_:
                    continue
                x = float(x)
                y = float(y)
            except:
                continue
            if x < grid_center_x - half_x: j = 0
            elif x > grid_center_x + half_x: j = 2
            else: j = 1
            if y < grid_center_y - half_y: i = 0
            elif y > grid_center_y + half_y: i = 2
            else: i = 1
            old = slots[i][j]
            if old < 0:
                slots[i][j] = obj_idx
                kind_grid[i][j] = kind
                continue
            old_kind = objects[old][0]
            if (kind =='B' and old_kind =='S')\
                or (kind == 'W' and old_kind =='B'):
                slots[i][j] = obj_idx
                kind_grid[i][j] = kind
                continue
            elif (kind =='S' and old_kind =='B')\
                or (kind == 'B' and old_kind =='W'):
                continue
            if self.if_danger(kind, self.obj_num_[kind]):
                continue
            if self.if_danger(old_kind, self.obj_num_[old_kind]):
                slots[i][j] = obj_idx
                kind_grid[i][j] = kind
                continue
            center_x = grid_center_x + (j - 1) * cell_x
            center_y = grid_center_y + (i - 1) * cell_y
            old_x = float(objects[old][1])
            old_y = float(objects[old][2])
            old_dx, old_dy = old_x - center_x, old_y - center_y
            new_dx, new_dy = x - center_x, y - center_y
            if new_dx * new_dx + new_dy * new_dy < old_dx * old_dx + old_dy * old_dy:
                displaced = old
                slots[i][j] = obj_idx
                kind_grid[i][j] = kind
            else:
                displaced = obj_idx
            best_i = -1
            best_j = -1
            best_dist2 = None
            for free_i in range(3):
                for free_j in range(3):
                    if slots[free_i][free_j] >= 0:
                        continue
                    dx = float(objects[displaced][1]) - (grid_center_x + (free_j - 1) * cell_x)
                    dy = float(objects[displaced][2]) - (grid_center_y + (free_i - 1) * cell_y)
                    dist2 = dx * dx + dy * dy
                    if best_dist2 is None or dist2 < best_dist2:
                        best_dist2 = dist2
                        best_i, best_j = free_i, free_j
            if best_i >= 0:
                slots[best_i][best_j] = displaced
                kind_grid[best_i][best_j] = objects[displaced][0]

        snapped = []
        for i in range(3):
            for j in range(3):
                obj_idx = slots[i][j]
                if obj_idx >= 0:
                    snapped.append((objects[obj_idx][0], grid_center_x + (j - 1) * cell_x,
                                    grid_center_y + (i - 1) * cell_y))
        return snapped
    def merge_nearby_same_kind(self,objects, threshold_near=10.0):
        merged = []
        threshold_far = threshold_near+10
        for kind, x, y in objects:
            match_idx = -1
            if self.obj_num_[kind]>1:
                for idx, (old_kind, old_x, old_y) in enumerate(merged):
                    if old_kind != kind:
                        continue
                    if self.last_side in ['U','D']:
                        object_dist = max(abs(y - self.my_car.y_current),abs(old_y - self.my_car.y_current))
                    else:object_dist = max(abs(x - self.my_car.x_current),abs(old_x - self.my_car.x_current))
                    if object_dist <= 30.0 or self.use_scan_point > 2:threshold = threshold_near
                    elif object_dist >= 110.0:threshold = threshold_far
                    else:
                        ratio = (object_dist - 30.0) / 80.0
                        threshold = threshold_near + (threshold_far - threshold_near) * ratio
                    dist2 = (x - old_x) ** 2 + (y - old_y) ** 2
                    if dist2 <= threshold ** 2:
                        match_idx = idx
                        break
            if match_idx < 0:
                merged.append((kind, x, y))
            else:
                old_kind, old_x, old_y = merged[match_idx]
                merged[match_idx] = (
                    old_kind,
                    (old_x + x) / 2.0,
                    (old_y + y) / 2.0,
                )
                self.obj_num_[old_kind]-=1
        return merged
    # 合并物体信息（双目视觉融合）
    def integrate_object_info(self,world_1,world_2):
        # 同一物体在两个扫描中的最大世界坐标偏差（cm）
        # 包含：测量噪声 ~5cm + 60cm 基线的视差效应 ~30cm + 安全裕量
        MATCH_DIST_THRESHOLD = 10.0
        # ── 2. 边界情况：任一侧为空则直接返回另一侧 ──
        if not world_1 and not world_2:
            return []
        if not world_1:
            return world_2[:]
        if not world_2:
            return world_1[:]
        groups_1 = {}
        groups_2 = {}
        for i, (kind,x, y) in enumerate(world_1):
            groups_1.setdefault(kind, []).append((i, x, y))
        for i, (kind,x, y) in enumerate(world_2):
            groups_2.setdefault(kind, []).append((i, x, y))
        matched_pairs = []
        used_1 = set()
        used_2 = set()
        all_kinds = set(groups_1.keys()) | set(groups_2.keys())
        for kind in all_kinds:
            objs_1 = groups_1.get(kind, [])
            objs_2 = groups_2.get(kind, [])
            if not objs_1 or not objs_2:
                continue
            candidates = []
            for i1, x1, y1 in objs_1:
                for i2, x2, y2 in objs_2:
                    d = (x1 - x2) ** 2 + (y1 - y2) ** 2
                    if d <= MATCH_DIST_THRESHOLD**2:
                        candidates.append((d, i1, i2))
            candidates.sort(key=lambda t: t[0])
            for d, i1, i2 in candidates:
                if i1 not in used_1 and i2 not in used_2:
                    matched_pairs.append((i1, i2))
                    used_1.add(i1)
                    used_2.add(i2)
        ob_info = []
        self.obj_num_['T'],self.obj_num_['S'],self.obj_num_['E'] = 0,0,0
        self.obj_num_['B'],self.obj_num_['W'] = 0,0
        for i1, i2 in matched_pairs:
            kind, x1, y1,  = world_1[i1]
            _, x2, y2 = world_2[i2]
            ob_info.append((kind,(x1 + x2) / 2.0, (y1 + y2) / 2.0))
            self.obj_num_[kind]+=1
        for i in range(len(world_1)):
            if i not in used_1:
                ob_info.append(world_1[i])
                kind,_,_ = world_1[i]
                self.obj_num_[kind]+=1
        for i in range(len(world_2)):
            if i not in used_2:
                ob_info.append(world_2[i])
                kind,_,_ = world_2[i]
                self.obj_num_[kind]+=1
        return ob_info
    
    def first_scan(self):
        def analyse_package(num, angle):
            global counter
            object_package=self.my_art_protocol.detect_objects_on_the_court()#[物体种类(ord),x,y]
            if object_package:
                counter +=1
                self.scan_empty_counter = 0
                new_world = self.handle_object_info(object_package, angle)
                if self.my_write_system.if_write_log:
                    self.my_write_system.write_str(f"detect{self.detected_num}:{new_world}, angle: {angle}\n")
                if self.now_objects: self.now_objects = self.integrate_object_info(self.now_objects,new_world)#将新帧与上一帧融合
                else: self.now_objects = new_world
                self.my_vision.analysed_objects = self.now_objects  
            else:
                self.scan_empty_counter += 1
                if self.scan_empty_counter>40:
                    self.my_plan.reset_navigate()
                    self.scan_waiting_count = 0 
                    self.scan_empty_counter = 0
                    self.my_order_manager.finish()
                    self.if_send_detect_message = False
                    self.if_plan_scan = False
                    counter = num#直接退出
            if counter == num:
                self.detected_num+=1#切换到下一个物体
                counter = 0
                self.scan_waiting_count = 0
                self.my_plan.reset_navigate()
                self.if_plan_scan = False
                self.my_order_manager.finish()
                self.if_send_detect_message = False
                #self.my_uart.write(f"{num}{self.my_vision.analysed_objects}\n")
                self.my_art_protocol.clear_uart_buffer()
        def scan_point(num):#输入帧数
            self.my_plan.navigate(path = self.planned_scan_path[self.detected_num][0],
                                  target_turn_angle = self.planned_scan_path[self.detected_num][1])
            if self.my_plan.if_finish_navigate:
                if self.scan_waiting_count < 5:
                    if not self.if_send_detect_message:
                        self.scan_empty_counter=0
                        self.if_send_detect_message = True
                        self.my_art_protocol.clear_uart_buffer()
                        self.my_order_manager.clear_knock()
                        self.my_order_manager.mode_detect()
                        if self.if_model_detect:
                            self.my_order_manager.trans_to_mode_detect()
                    self.scan_waiting_count +=1
                else:
                    analyse_package(num, self.planned_scan_path[self.detected_num][1])
        if self.detected_num == self.use_scan_point:
            # print(f"startmerge:{time.ticks_ms()}")
            self.now_objects = self.merge_nearby_same_kind(self.now_objects)
            self.now_objects = self.snap_objects_to_nine_grid(
                self.now_objects, self.SUDOKU_length_x, self.SUDOKU_width_y,
                maxNum=1, grid_center_x=self.data.center_x,
                grid_center_y=self.data.center_y)
            # print(f"endmerge:{time.ticks_ms()}")
            if self.if_back:
                if len(self.now_objects) != self.data.total_objects_num:
                    for i in range(len(self.now_objects)):
                        self.my_beep.test()
                        time.sleep_ms(300)
                    if self.my_write_system.if_write_log:
                        self.my_write_system.write_str(f"final_objects:{len(self.now_objects)}\n")
                        self.my_write_system.write_str(f"target{len(self.object_plan.target_objects)}\n")
                        self.my_write_system.write_str(f"path{len(self.object_plan.path)}\n")
                        self.my_write_system.write_str(f"score{len(self.object_plan.target_score)}\n")
                    self.exit()
                    return
            self.if_end_first_scan = True
        else:
            scan_point(self.scan_num)

    def handle_scan(self):
        global counter

        def path_extreme(points, mode):
            """从路径点列表中取指定坐标轴的最大值或最小值。

            :param points: 路径点列表，如 [[150, 90], [120, 10]]
            :param mode: 形如 'x_max' / 'x_min' / 'y_max' / 'y_min'，
                         首字符指定坐标轴(x/y)，后缀指定极值(max/min)
            :return: 对应坐标轴的最大值或最小值
            """
            idx = 0 if mode[0] == 'x' else 1
            values = [p[idx] for p in points]
            return max(values) if mode.endswith('max') else min(values)

        if not self.if_end_first_scan:
            if not self.if_plan_scan:
                start_point = [self.my_car.x_current, self.my_car.y_current + 40.0]
                if self.use_scan_point == 1:
                    target_point = self.fixed_scan_point[0][0] # type: ignore
                    target_angle = self.fixed_scan_point[0][1] # type: ignore
                    self.my_path.plan_path(target_point[0],target_point[1]) 
                    self.my_path.ready_path[-1] = target_point
                    self.planned_scan_path.append([self.my_path.ready_path, target_angle])
                    self.planned_scan_path[0][0].insert(0, start_point)
                    self.if_plan_scan = True
                    counter = 0
                    # 处理给从车发的消息
                    a = 0.0
                    slave_x = target_point[0]
                    slave_y = 20.0
                    self.my_main_protocol.send_path('P', a, (slave_x, slave_y))
                    self.if_plan_scan = True
                else:
                    if counter >= self.use_scan_point:
                        self.planned_scan_path[0][0].insert(0, start_point)
                        self.if_plan_scan = True
                        counter = 0

                        dist = 30.0
                        if self.use_scan_point == 4:
                            if self.scan_side == 'D' or self.scan_side == 'U':
                                slave_x = self.data.center_x
                                slave_y = self.max_pos + dist
                                a = 180
                            else:
                                slave_x = self.max_pos + dist
                                slave_y = self.data.center_y
                                a = -90
                        else:
                            a = self.fixed_scan_point[0][1] # type: ignore

                            if self.scan_side == 'D':
                                slave_x = self.data.center_x
                                slave_y = self.max_pos - dist
                            elif self.scan_side == 'L':
                                slave_x = self.max_pos - dist
                                slave_y = self.data.center_y
                            elif self.scan_side == 'R':
                                slave_x = self.max_pos + dist
                                slave_y = self.data.center_y
                            elif self.scan_side == 'U':
                                slave_x = self.data.center_x
                                slave_y = self.max_pos + dist

                        self.my_main_protocol.send_path('P', a, (slave_x, slave_y))
                        return
                    pt = self.fixed_scan_point[counter] # type: ignore
                    if counter > 0:
                        self.my_path.plan_path(pt[0][0], pt[0][1], start_point = self.fixed_scan_point[counter - 1][0]) # type: ignore
                    else:
                        self.my_path.plan_path(pt[0][0], pt[0][1], start_point = start_point) 
                    self.my_path.ready_path[-1] = pt[0]

                    if self.use_scan_point > 1 and counter == self.use_scan_point - 1:
                        if self.use_scan_point == 4:
                            if self.scan_side == 'D' or self.scan_side == 'U':
                                mode = 'y_max'
                            else:
                                mode = 'x_max'
                        else:
                            if self.scan_side == 'D':
                                mode = 'y_min'
                            elif self.scan_side == 'L':
                                mode = 'x_min'
                            elif self.scan_side == 'U':
                                mode = 'y_max'
                            else:
                                mode = 'x_max'

                        self.max_pos = path_extreme(self.my_path.ready_path, mode)

                    self.planned_scan_path.append([self.my_path.ready_path, pt[1]])
                    counter += 1
            else:self.first_scan()
        else:self.exit()

    def handle_servo(self):
        pass

    def handle_move(self):
        # if state == MOVE
        self.my_moving.moving()
        if self.my_moving.if_finish_move:
            self.exit()  # 退出当前状态，进入下一个状�?

    def handle_retreat(self):
        pass

    def handle_calibrate(self):
        pass

    def handle_adjust(self):
        if self.black_state == READY_NAVIGATE:
            if self.my_car.y_current < -65:self.black_angle = 57#下
            else:self.black_angle = 123#
            # 此时进入盲盒状态
            self.my_vision.if_in_blind = True
            self.my_vision.current_servo_object = 'S'
            self.my_order_manager.mode_target()
            self.my_art_protocol.send_object_kind(self.my_vision.current_servo_object)
            self.black_state = NAVIGATE
        elif self.black_state == NAVIGATE:
            if self.black_angle == 57:
                self.my_plan.navigate(path = [[60,-65-6]],target_turn_angle = self.black_angle)
            else:
                self.my_plan.navigate(path = [[60,-65+46]],target_turn_angle = self.black_angle)
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.my_vision.current_servo_object and self.angle_pid.if_finish_turn():
                self.my_vision.ready_servo_and_orbit(target_point, 'servo')
                self.my_vision.object_radius = 19.0
                self.my_vision.reset_servo_angle()
                self.my_plan.reset_navigate()
                self.black_state = SERVO
                return
        elif self.black_state == SERVO:
            if self.my_vision.if_finish_servo:
                if self.my_main_protocol.get_slave_state() == "get":
                    global counter
                    self.my_main_protocol.send_start()
                    self.my_vision.if_orbit_ready = False
                    self.my_vision.if_finish_orbit = False
                    self.black_state = ORBIT
                    counter = 1
            self.my_vision.visual_servo_control()
        elif self.black_state == ORBIT:
            global counter
            if self.black_angle == 57:
                if counter == 1:angle = 180 
                elif counter == 2:angle = -60
                else: angle = 60
            else:
                if counter == 1:angle = -120 
                elif counter == 2:angle = 0
                else: angle = 120
            if self.my_vision.if_finish_orbit:
                if counter == 3:self.exit()
                else:
                    counter += 1
                    self.my_vision.if_orbit_ready = False
                    self.my_vision.if_finish_orbit = False
            else:
                self.my_vision.orbit_control(angle)
    def handle_return(self):
        # if state == RETURN
        self.my_plan.navigate(path = self.my_path.ready_path)  # 返回起始�?
        # 主车行驶多远后给从车发送路径信�?
        dist_threshold = 15.0
        if self.my_plan.finished_dist >= dist_threshold and not self.if_send_path and not self.if_send_return_message:
            self.my_main_protocol.send_path('R', 999, self.data.fixed_point[4])  # 发送路径信息给从车
            self.if_send_path = True  # 设置标志位，避免重复发送路径信�?
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入停止状�?

    def handle_stop(self):
        # if state == STOP
        self.my_plan.stop()             
