from micropython import const
import math
import gc
import time

# 计数器
counter = 0

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

OutLine = const(1)

# 搬运控制类
class MoveControl:
    def __init__(self, flash_sys, beep, photo, uart, uart_debug, car, plan, path_plan, plan_data, vision_manager, state, slave_protocol, art_protocol, order_manager, tof, angle_pid):
        self.my_beep = beep
        self.my_photo = photo
        self.my_uart = uart
        self.uart_debug = uart_debug
        self.vision_manager = vision_manager
        self.my_plan = plan
        self.my_path = path_plan
        self.plan_data = plan_data
        self.my_car = car
        self.my_state = state
        self.my_slave_protocol = slave_protocol
        self.my_art_protocol = art_protocol
        self.my_order_manager = order_manager
        self.flash_sys = flash_sys
        self.my_tof = tof
        self.angle_pid = angle_pid
        self.next_orbit_angle = 0.0  # 下一环绕角度
        self.move_pt_buffer = []     # 搬运目标点缓冲区
        self.next_point = []     # 下一目标点
        self.special_push = False
        self.adjust_point = []   # 微调目标点
        self.move_dir = 0
        self.now_object_pt = []
        self.next_postion = 'l'
        self.if_first_orbit = False
        self.send_navigate_feed_back = False
        self.push_postion = [1,0]
        self.plan_path = []
        self.angle_T = self.flash_sys.find_value("angle_T")
        self.angle_S = self.flash_sys.find_value("angle_S")
        self.angle_B = self.flash_sys.find_value("angle_B")
        self.retreat_lenth = self.flash_sys.find_value("retreat_lenth")
        self.start_scan_range = self.flash_sys.find_value("start_scan_range")
        self.delay_more = False
        self.current_state = ORBIT  # 当前状态：0为环绕，1为视觉伺服，2为搬运， 3为微调
        self.if_send_to_main = False  # 是否向art发送完成信号
        self.if_finish_move = False  # 是否完成搬运
        self.if_get_orbit_angle = False  # 是否获取环绕角度
        self.if_to_the_top = False
        self.if_first_navigate = True
        self.navigate_buffer ={
                        'SLA_P':[],
                        'ANGLE':[0,0],
        }
        self.__angle=30
        self.surrounding_points = {
            'LD': [],
            'RD': [],
        }
        self.if_change_side = True
        gc.collect()
    def reset_orbit(self):
        self.if_get_orbit_angle = False
        self.vision_manager.if_orbit_ready = False
        self.vision_manager.if_finish_orbit = False
    def get_object_square_points(self,car_angle,L):#寻找物体周围点位
        a=L
        if car_angle == 0:
            forward = (0, 1)
            right = (1, 0)
        elif car_angle == 90:
            forward = (1, 0)
            right = (0, -1)
        elif car_angle == 180:
            forward = (0, -1)
            right = (-1, 0)
        elif car_angle == -90:
            forward = (-1, 0)
            right = (0, 1)
        else:
            raise ValueError("car_angle must be one of 0, 90, 180, -90")
        fx, fy = forward
        rx, ry = right
        lx, ly = -rx, -ry
        LD = [self.now_object_pt[0] + lx * a - fx * a, self.now_object_pt[1] + ly * a - fy * a]
        RD = [self.now_object_pt[0] + rx * a - fx * a, self.now_object_pt[1] + ry * a - fy * a]
        self.surrounding_points =  {
            'LD': LD,
            'RD': RD,
        }
    def judge_next_turn(self, sp,ref_yaw = 0):
        if sp == 'T':
            self.move_dir = 0
        elif sp in ['S','E']:self.move_dir = -90
        else: self.move_dir = 90
        angle = self.move_dir - ref_yaw
        if angle <= -180:angle += 360
        elif angle > 180:angle -= 360
        return angle
    def ready_move(self, target_ref_yaw_deg, point, sp,now_side = 'D'):
        self.delay_more = False
        self.send_navigate_feed_back = False
        self.record_angle = self.my_car.now_yaw  # 保持弧度制供 judge_next_turn 默认使用
        RECT = []
        center_x = self.plan_data.center_x
        center_y = self.plan_data.center_y
        lenth = self.plan_data.lenth
        self.special_push = False
        self.if_first_orbit = False
        if target_ref_yaw_deg > -45.0 and target_ref_yaw_deg <= 45.0: 
            target_side = 'D'
            if int(target_ref_yaw_deg) != 0:
                Num = int(target_ref_yaw_deg - 0)
                target_ref_yaw_deg = 0
                if now_side == 'L':p2 = [center_x - 1.5*lenth,point[1]-lenth*Num]
                else:p2 = [center_x + 1.5*lenth,point[1]-lenth*Num]
                RECT = [[min(point[0],p2[0]),min(point[1],p2[1])],[max(point[0],p2[0]),max(point[1],p2[1])]]
        elif target_ref_yaw_deg > 45.0 and target_ref_yaw_deg <= 135.0:
            target_side = 'L'
            if int(target_ref_yaw_deg) != 90:
                Num = int(target_ref_yaw_deg - 90)
                target_ref_yaw_deg = 90
                if Num < 4:
                    if now_side == 'U':p2 = [point[0]-lenth*Num,center_y + 1.5*lenth]
                    else:p2 = [point[0]-lenth*Num,center_y - 1.5*lenth]
                    RECT = [[min(point[0],p2[0]),min(point[1],p2[1])],[max(point[0],p2[0]),max(point[1],p2[1])]]
                else:#specialmove逻辑
                    self.special_push = True
                    cy = Num / 5 * lenth + center_y - lenth*2
                    half_h = 6 + self.plan_data.SAFE_MARGIN
                    rect = [(center_x - lenth*1.6,cy - half_h),
                            (point[0] - lenth*0.5,cy - half_h),
                            (point[0] - lenth*0.5,cy + half_h),
                            (center_x - lenth*1.6,cy + half_h),]
        elif target_ref_yaw_deg > 135.0 or target_ref_yaw_deg <= -135.0:
            target_side = 'U'
            if int(target_ref_yaw_deg) != 180:
                Num = int(target_ref_yaw_deg - 180)
                target_ref_yaw_deg = 180
                if now_side == 'L':p2 = [center_x - 1.5*lenth,point[1]-lenth*Num]
                else:p2 = [center_x + 1.5*lenth,point[1]-lenth*Num]
                RECT = [[min(point[0],p2[0]),min(point[1],p2[1])],[max(point[0],p2[0]),max(point[1],p2[1])]]
        elif target_ref_yaw_deg > -135.0 and target_ref_yaw_deg <= -45.0:
            target_side = 'R'
            if int(target_ref_yaw_deg) != -90:
                Num = int(target_ref_yaw_deg + 90)
                target_ref_yaw_deg = -90
                if now_side == 'U':p2 = [point[0]-lenth*Num,center_y + 1.5*lenth]
                else:p2 = [point[0]-lenth*Num,center_y - 1.5*lenth]
                RECT = [[min(point[0],p2[0]),min(point[1],p2[1])],[max(point[0],p2[0]),max(point[1],p2[1])]]            
        if now_side == 'D':current_ref_yaw_deg = 0
        elif now_side == 'L':current_ref_yaw_deg = 90.0
        elif now_side == 'R':current_ref_yaw_deg = -90.0
        else: current_ref_yaw_deg = 180
        if now_side != target_side:self.if_change_side = True
        else:self.if_change_side = False
        # 初始参考偏航角就是当前小车所在方向（度数）
        turn_angle = 0.0
        turn_angle = self.judge_next_turn(sp,target_ref_yaw_deg)
        target_turn = target_ref_yaw_deg + turn_angle
        # 角度限幅到 [-180, 180)
        target_turn = (target_turn + 180.0) % 360.0 - 180.0
        car_postion = target_turn
        if sp in ['S','E']:self.__angle = self.vision_manager.angle_S
        elif sp == 'T':self.__angle = self.vision_manager.angle_T
        else:self.__angle = self.vision_manager.angle_B
        angle_l=(target_turn + self.__angle + 180.0) % 360.0 - 180.0
        angle_r=(target_turn - self.__angle + 180.0) % 360.0 - 180.0
        angle_l0=(target_ref_yaw_deg + self.__angle + 180.0) % 360.0 - 180.0
        angle_r0=(target_ref_yaw_deg - self.__angle + 180.0) % 360.0 - 180.0
        self.if_to_the_top = False
        self.now_object_pt = point
        self.vision_manager.if_next_orbit = False
        self.get_object_square_points(target_ref_yaw_deg, 16)
        if now_side == target_side:
            self.vision_manager.if_next_orbit = True
            if self.next_postion == 'r':
                sla_p = self.surrounding_points['RD']
                angle0 = angle_r0
                angle = angle_r
                self.next_postion = 'l'
                self.my_tof.ready_tof('left', target_turn, sp)
                car_postion -= 90
            else:
                sla_p = self.surrounding_points['LD']
                angle0 = angle_l0
                angle = angle_l
                self.next_postion = 'r'
                self.my_tof.ready_tof('right', target_turn, sp)
                car_postion += 90
            if turn_angle == 0.0:
                self.vision_manager.if_next_orbit = False
                self.if_to_the_top =True
            elif turn_angle == 90.0:
                if self.next_postion == 'r':self.if_first_orbit = True
                else:self.if_first_orbit = False
            elif turn_angle == 180.0:
                if self.next_postion == 'r':
                    sla_p = self.surrounding_points['LD']
                    angle0 = angle_l0
                    angle = angle_r
                    car_postion -= 180
                    self.next_postion = 'l'
                    self.my_tof.ready_tof('left', target_turn, sp)
                else:
                    sla_p = self.surrounding_points['RD']
                    angle0 = angle_r0
                    angle = angle_l
                    car_postion += 180
                    self.next_postion = 'r'
                    self.my_tof.ready_tof('right', target_turn, sp)
                self.if_first_orbit = False
            elif turn_angle == -90.0:
                if self.next_postion == 'r':self.if_first_orbit = False
                else:self.if_first_orbit = True
            if self.special_push:
                self.plan_data.rectangles.insert(-1,rect)
            if now_side =='D': self.my_path.plan_path(sla_p[0],min(self.my_plan.plan_data.center_rect[0][1],sla_p[1]),ignore_center_rect=True)
            elif now_side =='U': self.my_path.plan_path(sla_p[0],max(self.my_plan.plan_data.center_rect[3][1],sla_p[1]),ignore_center_rect=True)
            elif now_side =='L': self.my_path.plan_path(min(self.my_plan.plan_data.center_rect[0][0],sla_p[0]),sla_p[1],ignore_center_rect=True)   
            else: self.my_path.plan_path(max(self.my_plan.plan_data.center_rect[3][0],sla_p[0]),sla_p[1],ignore_center_rect=True)
            self.my_path.ready_path.append(sla_p)
            if self.special_push:
                self.plan_data.rectangles.pop(-2)
        else:
            self.delay_more = True
            dicc = {'D':0,'L':1,'U':2,'R':3,}
            if self.if_first_navigate:
                if (dicc[target_side] - dicc[now_side]) % 4 == 1:#要到左侧
                    sla_p = self.surrounding_points['RD']
                    angle0 = angle_r0
                    angle = angle_r
                    self.next_postion = 'l'
                    self.my_tof.ready_tof('left', target_turn, sp)
                    car_postion -= 90      
                else:
                    sla_p = self.surrounding_points['LD']
                    angle0 = angle_l0
                    angle = angle_l
                    self.next_postion = 'r'
                    self.my_tof.ready_tof('right', target_turn, sp)
                    car_postion += 90
            else:
                if (dicc[target_side] - dicc[now_side]) % 4 == 1:#要到左侧
                    if self.next_postion == 'r':
                        sla_p = self.surrounding_points['RD']
                        angle0 = angle_r0
                        angle = angle_r
                        self.next_postion = 'l'
                        self.my_tof.ready_tof('left', target_turn, sp)
                        car_postion -= 90      
                    else:
                        sla_p = self.surrounding_points['LD']
                        angle0 = angle_l0
                        angle = angle_l
                        self.next_postion = 'r'
                        self.my_tof.ready_tof('right', target_turn, sp)
                        car_postion += 90
                else:
                    if self.next_postion == 'r':
                        sla_p = self.surrounding_points['RD']
                        angle0 = angle_r0
                        angle = angle_r
                        self.next_postion = 'l'
                        self.my_tof.ready_tof('left', target_turn, sp)
                        car_postion -= 90    
                    else:
                        sla_p = self.surrounding_points['LD']
                        angle0 = angle_l0
                        angle = angle_l
                        self.next_postion = 'r'
                        self.my_tof.ready_tof('right', target_turn, sp)
                        car_postion += 90
            self.if_to_the_top =True
            if not RECT:
                therhold = 7
                if target_side =='D':self.my_path.plan_path(sla_p[0],self.my_plan.plan_data.center_rect[0][1]-therhold)
                elif target_side =='U':self.my_path.plan_path(sla_p[0],self.my_plan.plan_data.center_rect[3][1]+therhold)
                elif target_side =='L':self.my_path.plan_path(self.my_plan.plan_data.center_rect[0][0]-therhold,sla_p[1])
                else: self.my_path.plan_path(self.my_plan.plan_data.center_rect[3][0]+therhold,sla_p[1])
            else:
                big_rect = [self.my_plan.plan_data.center_rect[0],self.my_plan.plan_data.center_rect[3]]
                p_2 = RECT[0][:]
                p_1 = sla_p[:]
                if target_side =='D': 
                    p_1[1] = RECT[0][1]
                elif target_side =='U': 
                    p_2[1] = RECT[1][1]
                    p_1[1] = RECT[1][1]
                elif target_side =='L': 
                    p_1[0] = RECT[0][0]
                else:
                    p_2[0] = RECT[1][0]
                    p_1[0] = RECT[1][0]
                if now_side =='D': p_2[1] = big_rect[0][1]
                elif now_side =='U': p_2[1] = big_rect[1][1]
                elif now_side =='L': p_2[0] = big_rect[0][0]
                else:p_2[0] = big_rect[1][0]
                self.my_path.plan_path(p_2[0],p_2[1])   
                self.my_path.ready_path.append(p_1)
            self.my_path.ready_path.append(sla_p)
        car_postion = 180 - (180 - car_postion) % 360
        if car_postion<=90+0.01 and car_postion>=90-0.01:self.push_postion = [1,0]
        elif car_postion<=0.01 and car_postion>=-0.01:self.push_postion = [0,1]
        elif car_postion<=-90+0.01 and car_postion>=-90-0.01:self.push_postion = [-1,0]
        else :self.push_postion = [0,-1]
        nowx,nowy= self.my_car.x_current,self.my_car.y_current
        max_x,max_y = self.plan_data.FIELD_W - self.retreat_lenth,self.plan_data.FIELD_H - self.retreat_lenth
        min_x,min_y = self.retreat_lenth,self.retreat_lenth
        if nowx > max_x or nowx < min_x or nowy > max_y or nowy < min_y:
            self.plan_path.insert(0,[max(min_x,min(max_x,nowx)),max(min_y,min(max_y,nowy))])
        self.navigate_buffer={
                    'SLA_P':self.my_path.ready_path,
                    'ANGLE':[angle0,angle],
                }
        self.if_get_orbit_angle=True
        self.current_state = NAVIGATE
        self.if_finish_move = False
        self.if_send_to_main = False
        self.my_plan.reset_navigate()
        self.my_plan.reset_navigate_angle()
        # print(f"{self.if_first_orbit}, {self.if_change_side}, {self.next_postion}, {now_side}, {target_side}, {target_ref_yaw_deg}, {turn_angle}, {self.navigate_buffer['SLA_P']}, {self.navigate_buffer['ANGLE']}, {self.push_postion}")
    # 重置小车里程计
    def reset_car_pos(self):
        current_object = self.vision_manager.current_servo_object
        # 经验修正值
        correction = 2.0
        if current_object == 'T':
            self.my_car.y_current = self.plan_data.FIELD_H - correction
        elif current_object in ['S', 'E']:
            self.my_car.x_current = 0.0 + correction
        elif current_object in ['B', 'W']:
            self.my_car.x_current = self.plan_data.FIELD_W - correction
    def caculate_move_path(self,path):
        try:
            dx1=path[2][0]
            dy1=path[2][1]
            if path[1] == 0:
                pl=[self.my_car.x_current+dx1,self.plan_data.FIELD_H+30]
            elif path[1] == 180:
                pl=[self.my_car.x_current+dx1,-30]
            elif path[1] == 90:
                pl=[self.plan_data.FIELD_W+30,self.my_car.y_current+dy1]
            elif path[1] == -90:
                pl=[-30,self.my_car.y_current+dy1]
            p_m = [self.my_car.x_current + dx1,
                   self.my_car.y_current + dy1,]
            if self.push_postion[0] != 0:
                total_dy = abs(pl[1] - self.my_car.y_current)
                if total_dy <= 1e-6:return []
                self.my_plan.keep_x_or_y_v = False
            elif self.push_postion[1] != 0:
                # y 内收，保持 vx
                total_dx = abs(pl[0] - self.my_car.x_current)
                if total_dx <= 1e-6:return []
                self.my_plan.keep_x_or_y_v = True
            else:return []
            if dx1==0 and dy1==0:
                return [pl]
            return [p_m,pl]
        except:return []
    # 状态过渡函数
    def state_transition(self):
        global counter
        if self.current_state == NAVIGATE:
            if self.vision_manager.if_send_order == False:
                self.my_order_manager.mode_target()
                self.my_art_protocol.send_object_kind(self.vision_manager.current_servo_object)
                self.my_art_protocol.clear_uart_buffer()
                self.vision_manager.if_send_order = True
            if not self.send_navigate_feed_back:
                self.my_slave_protocol.send_slave_state("ready")
            if self.if_first_navigate:
                self.if_first_navigate = False
            self.plan_path = []
            self.vision_manager.if_finish_servo = False
            self.vision_manager.if_lost_object = True
            self.current_state = SCAN
            return 
        elif self.current_state == SCAN:
            if self.my_plan.if_finish_navigate:
                self.current_state = SERVO
                self.vision_manager.reset_servo_angle()
                self.my_plan.reset_navigate()
                self.my_plan.reset_navigate_angle()
                self.reset_orbit() # 重置环绕相关变量
                self.plan_path = []
                self.vision_manager.if_lost_object = True
                return
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.vision_manager.current_servo_object and self.angle_pid.if_finish_turn():
                points = self.vision_manager.calc_object_global_pos(target_point[0], target_point[1], self.vision_manager.current_servo_object)
                if self.vision_manager.if_in_rect(points[0], points[1]):
                    self.vision_manager.ready_servo_and_orbit(chr(target_point[2]), 'servo', target_point)
                    self.vision_manager.reset_servo_angle()
                    self.my_plan.reset_navigate()
                    self.my_plan.reset_navigate_angle()
                    self.reset_orbit() # 重置环绕相关变量
                    self.plan_path = []
                    self.current_state = SERVO
            return
        elif self.current_state == ORBIT:
            # 延时200ms
            counter += 1
            if counter <= 20:
                return 
            if self.if_send_to_main == False:
                # 通知主车已完成当前环绕
                self.my_slave_protocol.send_slave_state("finish")
                self.if_send_to_main = True
                self.my_plan.fitting_path_ = []
            if self.vision_manager.if_send_order == True:
                self.my_order_manager.finish() # 关闭目标识别模式
                self.vision_manager.if_send_order = False
            rec_path = self.my_slave_protocol.get_path_list()
            if rec_path and rec_path[0] == 'R':return
            if rec_path and rec_path[0] == 'M':
                self.plan_path = self.caculate_move_path(rec_path)
                if not self.plan_path:return
                # 重置计数器
                counter = 0
                self.my_slave_protocol.send_slave_state("ready")
                self.vision_manager.if_finish_servo = False
                self.if_send_to_main = False
                self.my_plan.reset_navigate()
                self.my_plan.reset_navigate_angle()
                if self.vision_manager.current_servo_object in ['S','E']:
                    if rec_path[2][1] > 1e-3 and self.next_postion == 'r':self.my_plan.if_inside_sandbag = True
                    elif rec_path[2][1] < 1e-3 and self.next_postion == 'l':self.my_plan.if_inside_sandbag = True
                self.my_plan.move_state = MOVE
                self.current_state = MOVE

                # 如果当前伺服物体存在，则设置 my_plan 的 if_push_T 标志为 True
                if self.vision_manager.current_servo_object == 'T':
                    self.my_plan.if_push_T = True
        elif self.current_state == MOVE:
            if self.my_plan.if_near_line or self.my_plan.if_finish_navigate:
                self.my_tof.reset_tof()
                self.my_photo.reset_photo()
                self.my_plan.reset_navigate()
                self.my_plan.if_near_line = False
                self.if_finish_move = True
                self.my_plan.fitting_path_ = []
                self.my_plan.move_state = NAVIGATE
                self.current_state = NAVIGATE  
        elif self.current_state == ADJUST:
            pass
        elif self.current_state == SERVO:
            if self.vision_manager.if_lost_object:
                # 若丢失物体则给从车发送丢失消息
                self.my_slave_protocol.send_slave_state("lost")
                self.if_finish_move = True
                return
            if self.if_to_the_top:
                self.vision_manager.if_finish_servo = False
                self.vision_manager.if_finish_orbit = False
                self.vision_manager.if_orbit_ready = False
                self.vision_manager.reset_orbit_angle()
                self.vision_manager.if_finish_orbit = True#直接跳过旋转
                self.current_state = ORBIT
            elif self.if_first_orbit:
                self.vision_manager.if_finish_servo = False
                self.vision_manager.if_finish_orbit = False
                self.vision_manager.if_orbit_ready = False
                self.vision_manager.reset_orbit_angle()
                self.my_slave_protocol.send_slave_state("get")
                self.current_state = ORBIT
            elif self.my_slave_protocol.get_start_signal():
                self.vision_manager.if_finish_servo = False
                self.vision_manager.if_finish_orbit = False
                self.vision_manager.if_orbit_ready = False
                self.vision_manager.reset_orbit_angle()
                self.current_state = ORBIT
            return 
    # 搬运控制函数
    def moving(self):
        if self.if_finish_move:
            return
        if self.current_state == NAVIGATE:
            NAV_T=self.navigate_buffer
            if self.if_change_side:#若换边，打开边走边转，旋转大角度
                self.my_plan.navigate(NAV_T['SLA_P'], NAV_T['ANGLE'][0],if_high_angle=False,if_first_turn=False)
            else:
                if self.if_first_navigate:self.my_plan.navigate(NAV_T['SLA_P'], NAV_T['ANGLE'][0],if_high_angle=False,if_first_turn=False)
                else:self.my_plan.navigate(NAV_T['SLA_P'], NAV_T['ANGLE'][0],if_high_angle=True,if_first_turn=False)
            if self.delay_more and len(self.my_plan.path) >=4:
                _delay = 15 + (len(self.my_plan.path)-3)*15
            else:_delay = 15
            if self.send_navigate_feed_back == False and self.my_plan.finished_dist >= _delay:
                self.send_navigate_feed_back = True
                self.my_slave_protocol.send_slave_state("ready")
            if (self.my_plan.aimed_point_index == len(self.my_plan.path) - 2) and self.my_plan.rest_dist <= self.start_scan_range:
                self.state_transition()
                return
        elif self.current_state == ORBIT:
            if self.vision_manager.if_finish_orbit:
                self.state_transition() # 退出当前状态，进入搬运状态
                return
            NAV_T=self.navigate_buffer
            # print(f"{self.navigate_buffer}")
            self.vision_manager.orbit_control(NAV_T['ANGLE'][1])
        elif self.current_state == SCAN:
            NAV_T=self.navigate_buffer
            self.my_plan.navigate(path = [NAV_T['SLA_P'][-1]])
            self.state_transition()
        elif self.current_state == MOVE:
            self.my_photo.update_photo_state()
            if self.my_photo.current_state == OutLine:
                self.reset_car_pos()
                self.my_photo.reset_photo()
                self.my_beep.test()
                self.my_plan.if_near_line = True
                self.my_plan.if_finish_navigate = True
            self.my_plan.navigate(path = self.plan_path)
            self.my_tof.dist_control()
            if self.my_plan.if_finish_navigate == True:
                self.state_transition()
        elif self.current_state == ADJUST:
            self.vision_manager.visual_servo_control()
            if self.vision_manager.if_finish_servo == True:
                self.state_transition()
        elif self.current_state == SERVO:
            if self.vision_manager.if_finish_servo or self.my_plan.if_finish_navigate:
                self.state_transition()  # 退出当前状态
            if self.vision_manager.if_lost_object == False:
                self.vision_manager.visual_servo_control()
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
                self.my_plan.navigate(path = [[right_x, right_y], [left_x, left_y]])
                target_point = self.my_art_protocol.coordinate_receive()
                # 判断红色沙包是否在矩形框内
                if target_point:
                    if chr(target_point[2]) == 'S':
                        actual_point = self.vision_manager.predict_point(target_point[0], target_point[1])
                        if_red_valid = self.vision_manager.if_in_rect(actual_point[0], actual_point[1])
                    else:
                        if_red_valid = True
                if target_point and chr(target_point[2]) == self.vision_manager.current_servo_object and if_red_valid:
                    self.vision_manager.ready_servo_and_orbit(chr(target_point[2]), 'servo',point = [target_point[0],target_point[1]])
                    self.my_plan.reset_navigate()
                    self.vision_manager.if_lost_object = False
