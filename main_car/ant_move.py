from micropython import const
import math
import gc

PI = const(3.1415926)
READY_NAVIGATE = const(0)   # 准备导航状态
NAVIGATE = const(1)       # 导航状态
SCAN = const(2)           # 扫描状态
SERVO = const(3)          # 视觉伺服状态
ORBIT = const(4)          # 环绕状态
MOVE = const(5)           # 搬运状态
CALIBRATE = const(6)      # 校准状态
ADJUST = const(7)           # 微调状态
RETURN = const(8)		    # 返回状态
STOP = const(9)           # 停止状态
InField = const(-1)
OnLine = const(0)
OutLine = const(1)
# 多路复用器计数器
counter = 0
class MoveControl:
    def __init__(self,my_write_system,flash_sys, beep, photo, car, plan,path, plan_data,move_plan, vision_manager, state, main_protocol, art_protocol, order_manager,my_uart, angle_pid,obj_paln):
        self.my_write_system = my_write_system
        self.my_beep = beep
        self.my_photo = photo
        self.vision_manager = vision_manager
        self.my_obj_plan = obj_paln
        self.my_plan = plan
        self.my_path = path
        self.plan_data = plan_data
        self.my_car = car
        self.my_state = state
        self.my_main_protocol = main_protocol
        self.my_art_protocol = art_protocol
        self.my_order_manager = order_manager
        self.move_plan = move_plan
        self.my_uart = my_uart
        self.angle_pid = angle_pid
        self.now_object_pt = [0.0, 0.0]
        self.record_angle = 0.0  # 记录的角度(记录小车的最初的角度)
        self.flash_sys = flash_sys
        self.if_delay_more = False
        self.__angle=30
        self.angle_T = self.flash_sys.find_value("angle_T")
        self.angle_S = self.flash_sys.find_value("angle_S")
        self.angle_B = self.flash_sys.find_value("angle_B")
        self.start_scan_range = self.flash_sys.find_value("start_scan_range")
        self.surrounding_points = {'LD': [],'RD': [],}
        self.if_first_orbit = False
        self.slave_message_delay = 15
        self.now_barriar = []
        self.angle_buffer = []   # 角度缓冲区
        self.move_dir = 0 #保存推动方向
        self.if_to_the_top =False
        self.if_change_side = False
        self.next_postion = 'r'#下次主车推完返回时相对于从车的位置
        self.current_state = ORBIT  # 当前状态：0为环绕，1为视觉伺服，2为搬运， 3为微调
        self.if_send_orbit_command = False  # 是否发送过环绕控制指令
        self.if_send_navigate_command = False  # 是否发送过惯导控制指令
        self.if_start_orbit = False  # 是否开始环绕
        self.if_finish_move = False  # 是否完成搬运
        self.if_slave_ready_move = False
        self.if_first_navigate = True # 是否第一次惯导
        self.push_postion = [0,0] #用于判断推动时所需的xy补偿
        self.plan_path = []
        self.send_point = []
        self.sidenum_dicc = {'D':0,'L':1,'U':2,'R':3,}
        self.run_first = True
        self.get_slave_navigate_state = False
        self.saved_best_path = []
        self.navigate_buffer = {
            'MAIN_P':[],
            'SLA_P':[],
            'ANGLE':0,
        }
        gc.collect()
    # 判断小车编队到下一目标点时的转向
    def judge_next_turn(self, current_turn, sp):
        if sp == 'T':
            self.move_dir = 0
        elif sp in ['S','E']:self.move_dir = -90
        else: self.move_dir = 90
        angle = self.move_dir - current_turn
        if angle <= -180:angle += 360
        elif angle > 180:angle -= 360
        return angle
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
        else:raise ValueError("car_angle must be one of 0, 90, 180, -90")
        fx, fy = forward
        rx, ry = right
        lx, ly = -rx, -ry
        LD = [self.now_object_pt[0] + lx * a - fx * a, self.now_object_pt[1] + ly * a - fy * a]
        RD = [self.now_object_pt[0] + rx * a - fx * a, self.now_object_pt[1] + ry * a - fy * a]
        self.surrounding_points =  {
            'LD': LD,
            'RD': RD,
        }
    # 搬运前的准备
    def ready_move(self,point,now_side = 'D',target_side = 'D',RECT = [],Num = 0):
        '''now_side通过小车上次推动物体导出为小车现处边界 target_side通过路径规划函数给出为小车目标servo进入方向'''
        '''RECT为多用矩阵 为换边或specialmove时规划构造 Num用于换边或specialmove时与从车协同发送合适消息'''
        # print("[mem] ready_move free:{} alloc:{}".format(gc.mem_free(), gc.mem_alloc()))
        self.if_delay_more = False
        if not point or len(point) < 2:return False
        #self.now_object_pt = self.vision_manager.calc_object_global_pos(point[0],point[1])
        self.now_object_pt = point[:]
        if not self.vision_manager.if_in_rect(self.now_object_pt[0],self.now_object_pt[1]):return False
        self.current_state = NAVIGATE
        self.if_finish_move = False
        self.if_start_orbit = False
        self.if_slave_ready_move = False
        self.navigate_buffer.clear()
        self.my_plan.reset_navigate()
        self.my_plan.reset_navigate_angle()
        if self.vision_manager.current_servo_object in ['S','E']:self.__angle = self.angle_S
        elif self.vision_manager.current_servo_object == 'T':self.__angle = self.angle_T
        else:self.__angle = self.angle_B
        # 记录小车当前角度
        if now_side == 'L':current_turn_deg = 90.0
        elif now_side == 'R':current_turn_deg = -90.0
        elif now_side == 'D':current_turn_deg = 0.0
        else:current_turn_deg = 180
        if target_side == now_side:  
            self.if_change_side = False
            target_turn_deg = current_turn_deg
        else:
            self.if_change_side = True
            if target_side == 'L':target_turn_deg = 90.0
            elif target_side == 'R':target_turn_deg = -90.0
            elif target_side == 'D':target_turn_deg = 0.0
            else:target_turn_deg = 180
        self.angle_buffer.clear()
        self.get_object_square_points(target_turn_deg,16)
        '''turn_angle为小车视角下认为servo后应该如何转向'''
        turn_angle = self.judge_next_turn(target_turn_deg,self.vision_manager.current_servo_object)
        '''turn_angle为世界坐标下认为servo后应该如何转向'''
        target_turn = target_turn_deg + turn_angle
        # 角度限幅到 [-180, 180)
        target_turn = (target_turn + 180.0) % 360.0 - 180.0
        car_postion = target_turn
        '''angle0为servo时目标脚 angle为servo时目标脚'''
        angle_l0=(target_turn_deg + self.__angle + 180.0) % 360.0 - 180.0
        angle_r0=(target_turn_deg - self.__angle + 180.0) % 360.0 - 180.0
        angle_l=(target_turn + self.__angle + 180.0) % 360.0 - 180.0
        angle_r=(target_turn - self.__angle + 180.0) % 360.0 - 180.0
        S_PAth = [self.vision_manager.current_servo_object,self.now_object_pt]
        self.run_first = True
        self.vision_manager.if_next_orbit = False
        if self.if_change_side:
            '''若换边'''
            self.if_delay_more = True
            if self.if_first_navigate:
                '''若第一次'''
                if (self.sidenum_dicc[target_side] - self.sidenum_dicc[now_side]) % 4 == 1:#要到左侧
                    m_PAth = self.surrounding_points['LD']
                    ANGle = [angle_l0,target_turn_deg+Num,angle_l]
                    car_postion += 90
                    self.next_postion = 'r'
                else:
                    m_PAth = self.surrounding_points['RD']
                    ANGle = [angle_r0,target_turn_deg+Num,angle_r]
                    car_postion -= 90
                    self.next_postion = 'l'
            else:
                if (self.sidenum_dicc[target_side] - self.sidenum_dicc[now_side]) % 4 == 1:
                    '''换边左侧进入'''
                    if self.next_postion == 'r':
                        self.run_first = False
                        self.get_slave_navigate_state = False
                        m_PAth = self.surrounding_points['RD']
                        ANGle = [angle_r0,target_turn_deg+Num,angle_r]
                        car_postion -= 90
                        self.next_postion = 'l'
                    else:
                        m_PAth = self.surrounding_points['LD']
                        ANGle = [angle_l0,target_turn_deg+Num,angle_l]
                        car_postion += 90
                        self.next_postion = 'r'
                else:
                    '''换边右侧进入'''
                    if self.next_postion == 'l':
                        self.run_first = False
                        self.get_slave_navigate_state = False
                        m_PAth = self.surrounding_points['LD']
                        ANGle = [angle_l0,target_turn_deg+Num,angle_l]
                        car_postion += 90
                        self.next_postion = 'r'
                    else:
                        m_PAth = self.surrounding_points['RD']
                        ANGle = [angle_r0,target_turn_deg+Num,angle_r]
                        car_postion -= 90
                        self.next_postion = 'l'
            self.if_to_the_top =True
            if not RECT:
                '''没有RECT启用大矩形框'''
                therhold = 7
                if target_side =='D':self.my_path.plan_path(m_PAth[0],self.my_plan.plan_data.center_rect[0][1]-therhold)
                elif target_side =='U':self.my_path.plan_path(m_PAth[0],self.my_plan.plan_data.center_rect[3][1]+therhold)
                elif target_side =='L':self.my_path.plan_path(self.my_plan.plan_data.center_rect[0][0]-therhold,m_PAth[1])   
                else:self.my_path.plan_path(self.my_plan.plan_data.center_rect[3][0]+therhold,m_PAth[1])
            else:
                '''有RECT解出路径点'''
                big_rect = [self.my_plan.plan_data.center_rect[0],self.my_plan.plan_data.center_rect[3]]
                p_2 = RECT[0][:]#倒数第二个中继点
                p_1 = m_PAth[:]#倒数第一个中继点
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
                # self.my_path.ready_path[-1] = (p_2[0],p_2[1]) 
                self.my_path.ready_path.append(p_1)
            self.my_path.ready_path.append(m_PAth)
        else:
            '''若不换边'''
            self.vision_manager.if_next_orbit = True
            if not self.if_first_navigate:
                '''通过点位左右和主从车相对位置判断主从车谁先行'''
                if now_side == 'D':
                    if (point[0]-self.my_car.x_current > 0 and self.next_postion == 'l') or\
                    (point[0]-self.my_car.x_current < 0 and self.next_postion == 'r'):
                        self.run_first = False
                        self.get_slave_navigate_state = False
                elif now_side == 'U':
                    if (point[0]-self.my_car.x_current > 0 and self.next_postion == 'r') or\
                        (point[0]-self.my_car.x_current < 0 and self.next_postion == 'l'):
                        self.run_first = False
                        self.get_slave_navigate_state = False
                elif now_side == 'L':
                    if (point[1]-self.my_car.y_current > 0 and self.next_postion == 'r') or\
                        (point[1]-self.my_car.y_current < 0 and self.next_postion == 'l'):
                        self.run_first = False
                        self.get_slave_navigate_state = False
                elif now_side == 'R':
                    if (point[1]-self.my_car.y_current > 0 and self.next_postion == 'l') or\
                        (point[1]-self.my_car.y_current < 0 and self.next_postion == 'r'):
                        self.run_first = False
                        self.get_slave_navigate_state = False  
            if self.next_postion == 'r':
                m_PAth = self.surrounding_points['RD']
                ANGle = [angle_r0,target_turn_deg,angle_r]
                car_postion -= 90
                self.next_postion = 'l'
            else:
                m_PAth = self.surrounding_points['LD']
                ANGle = [angle_l0,target_turn_deg,angle_l]
                car_postion += 90
                self.next_postion = 'r'
            if turn_angle == 0.0:
                self.vision_manager.if_next_orbit = False
                if self.my_obj_plan.special_push:ANGle[1] = target_turn_deg + 5 * Num
                self.if_to_the_top =True
            elif turn_angle == 90.0:
                if self.next_postion == 'r':self.if_first_orbit = True
                else:self.if_first_orbit = False
            elif turn_angle == 180.0:
                if self.next_postion == 'r':
                    m_PAth = self.surrounding_points['LD']
                    ANGle = [angle_l0,target_turn_deg,angle_r]
                    car_postion -= 180
                    self.next_postion = 'l'
                else:
                    m_PAth = self.surrounding_points['RD']
                    ANGle = [angle_r0,target_turn_deg,angle_l]
                    car_postion += 180
                    self.next_postion = 'r'
                self.if_first_orbit = True
            elif turn_angle == -90.0:
                if self.next_postion == 'r':self.if_first_orbit = False
                else:self.if_first_orbit = True
            if self.my_obj_plan.special_push:
                self.plan_data.rectangles.insert(-1,RECT)
            if target_side =='D':self.my_path.plan_path(m_PAth[0],self.my_plan.plan_data.center_rect[0][1],ignore_center_rect=True)
            elif target_side =='U':self.my_path.plan_path(m_PAth[0],self.my_plan.plan_data.center_rect[3][1],ignore_center_rect=True)
            elif target_side =='L':self.my_path.plan_path(self.my_plan.plan_data.center_rect[0][0],m_PAth[1],ignore_center_rect=True)   
            else:self.my_path.plan_path(self.my_plan.plan_data.center_rect[3][0],m_PAth[1],ignore_center_rect=True)
            self.my_path.ready_path.append(m_PAth)
            if self.my_obj_plan.special_push:
                self.plan_data.rectangles.pop(-2)
        # print(f"if_change_side:{self.if_change_side}, RECT: {RECT}, if_first_navigate: {self.if_first_navigate},  ready_path: {self.my_path.ready_path}")
        car_postion = 180 - (180 - car_postion) % 360
        if car_postion<=90+0.01 and car_postion>=90-0.01:self.push_postion = [1,0]
        elif car_postion<=0.01 and car_postion>=-0.01:self.push_postion = [0,1]
        elif car_postion<=-90+0.01 and car_postion>=-90-0.01:self.push_postion = [-1,0]
        else :self.push_postion = [0,-1]
        self.navigate_buffer={
                        'MAIN_P':self.my_path.ready_path,
                        'SLA_P':S_PAth,
                        'ANGLE':ANGle,
                    }
        # print(f"{self.if_first_orbit}, {self.if_change_side}, {self.next_postion}, {now_side}, {target_side}, {target_turn_deg}")
        if not self.navigate_buffer:
            return False
        return True
    # 重置环绕控制标志位
    def reset_orbit(self):
        self.if_send_orbit_command = False
        self.if_start_orbit = False
        self.vision_manager.if_orbit_ready = False
        self.vision_manager.if_finish_orbit = False
        self.my_plan.reset_navigate()
    
    # 重置搬运控制相关变量
    def reset_move(self):
        self.if_slave_ready_move = False
        self.saved_best_path = []
        self.angle_buffer.clear()
        self.surrounding_points.clear()
        self.navigate_buffer.clear()
        self.current_state = NAVIGATE
        self.reset_orbit()
        self.if_send_navigate_command = False 
        self.if_finish_move = False
        self.if_to_the_top =False
        gc.collect()
    
    # 重置小车里程计
    def reset_car_pos(self):
        current_object = self.vision_manager.current_servo_object
        # 经验修正值
        correction = 2.0
        if current_object == 'T':
            self.my_car.y_current = self.plan_data.FIELD_H - correction
        elif current_object in ['S', 'E']:
            raw_x = self.my_car.x_current
            self.my_car.x_current = 0.0 + correction
        elif current_object in ['B', 'W']:
            self.my_car.x_current = self.plan_data.FIELD_W - correction
    def calculate_move_path(self):
        objects=self.now_barriar
        if self.move_dir==0 or self.move_dir==180:
            if self.my_car.now_yaw>0:swell_dir=-90
            else:swell_dir=90
        elif self.move_dir==-90 or self.move_dir==90:
            if self.my_car.now_yaw>-PI/2 and self.my_car.now_yaw<PI/2:swell_dir=180
            else:swell_dir=0
        else: return False
        plan_path = self.move_plan.plan_move(self.move_dir,swell_dir,objects,limit_angle = 55,generate_new_obj =True)
        used_path = 2
        if not plan_path or len(plan_path)<=1:
            try:
                used_path = 1
                dx,dy = self.saved_best_path
                p0 = [self.my_car.x_current,self.my_car.y_current]
                p1 = [self.my_car.x_current+dx,self.my_car.y_current+dy]
                if self.move_dir==0:p2 = [self.my_car.x_current+dx,self.plan_data.FIELD_H+30]
                elif self.move_dir==180:p2 = [self.my_car.x_current+dx,-30]
                elif self.move_dir==-90:p2 = [-30,self.my_car.y_current+dy]
                else :p2 = [self.plan_data.FIELD_W+30,self.my_car.y_current+dy]
                if abs(dx)<1e-3 or abs(dy)<1e-3:plan_path = [p0,p2]
                else:plan_path = [p0,p1,p2]
            except:return False
        try:
            dy = abs(plan_path[-1][1]-plan_path[0][1])
            dx = abs(plan_path[-1][0]-plan_path[0][0])
        except:return False
        if self.push_postion[0] == 0:
            self.my_plan.keep_x_or_y_v = True
        elif self.push_postion[1] == 0:
            self.my_plan.keep_x_or_y_v = False
        else: return False
        if len(plan_path) == 2:
            self.send_point=[0,0]
            self.plan_path = plan_path[1:]
        elif len(plan_path) == 3:
            self.send_point=[plan_path[1][0]-self.my_car.x_current,plan_path[1][1]-self.my_car.y_current]
            self.plan_path = plan_path[1:]
        if self.my_write_system.if_write_log:
            self.my_write_system.write_str(f"used path:{used_path},dx:{self.send_point[0]},dy:{self.send_point[1]},car_x:{self.my_car.x_current},car_y:{self.my_car.y_current},push_pos:{self.push_postion}\n")

        return True 
    # 状态过渡函数
    def state_transition(self):
        global counter
        # print("[mem] state{} free:{} alloc:{}".format(self.current_state, gc.mem_free(), gc.mem_alloc()))
        if self.current_state == NAVIGATE:
            if self.if_first_navigate:
                self.if_first_navigate = False
            self.my_art_protocol.clear_uart_buffer()
            NAV_T=self.navigate_buffer
            self.vision_manager.reset_servo_angle()
            self.vision_manager.ready_servo_and_orbit([self.now_object_pt[0],self.now_object_pt[1],ord(NAV_T['SLA_P'][0])])
            if self.vision_manager.if_send_order == False:#若还未打开摄像头
                # 打开摄像头
                self.my_order_manager.mode_target()
                self.my_art_protocol.send_object_kind(self.vision_manager.current_servo_object)
                self.vision_manager.if_send_order = True
            if self.if_send_navigate_command == False:
                self.if_send_navigate_command = True
                #self.my_main_protocol.send_path('P',NAV_T['ANGLE'][1],[-1,-1])
            if self.if_send_orbit_command == False:
                self.my_main_protocol.send_path(NAV_T['SLA_P'][0],NAV_T['ANGLE'][1],NAV_T['SLA_P'][1])
                self.if_send_orbit_command = True
            self.current_state = SCAN
        elif self.current_state == SCAN:
            if self.my_plan.if_finish_navigate:
                self.current_state = SERVO
                self.vision_manager.reset_servo_angle()
                self.my_plan.reset_navigate()
                self.reset_orbit() # 重置环绕相关变量
                self.plan_path = []
                self.vision_manager.if_lost_object = True
                return
            target_point = self.my_art_protocol.coordinate_receive()

            if target_point and chr(target_point[2]) == self.vision_manager.current_servo_object and self.angle_pid.if_finish_turn():
                real_point = self.vision_manager.predict_point(target_point[0], target_point[1],limit_y = None)
                if self.vision_manager.if_in_rect(real_point[0], real_point[1]):
                    self.vision_manager.ready_servo_and_orbit(target_point, 'servo')
                    self.vision_manager.reset_servo_angle()
                    self.my_plan.reset_navigate()
                    self.reset_orbit() # 重置环绕相关变量
                    self.plan_path = []
                    self.current_state = SERVO
                    return
        elif self.current_state == ORBIT:
            self.vision_manager.if_send_order = False
            if counter >= 5:
                order = self.my_main_protocol.get_slave_state()
                if self.if_slave_ready_move:
                    if order == "ready":
                        counter = 0
                        self.reset_orbit()  # 重置环绕相关变量
                        self.my_beep.test()
                        self.my_plan.reset_navigate_angle()
                        self.my_plan.reset_navigate()
                        if self.vision_manager.current_servo_object in ['S','E']:
                            if self.send_point[1] > 1e-3 and self.next_postion == 'r':self.my_plan.if_inside_sandbag = True
                            elif self.send_point[1] < 1e-3 and self.next_postion == 'l':self.my_plan.if_inside_sandbag = True
                        self.my_plan.move_state = MOVE
                        self.current_state = MOVE
                        self.vision_manager.if_finish_servo = False

                        # 如果当前伺服物体存在，则设置 my_plan 的 if_push_T 标志为 True
                        if self.vision_manager.current_servo_object == 'T':
                            self.my_plan.if_push_T = True
                else:
                    if order == "finish":
                        self.my_plan.fitting_path_ = []
                        if not self.calculate_move_path():
                            self.if_finish_move = True
                            return #直接退出return
                        self.my_main_protocol.send_path('M',self.move_dir,self.send_point)
                        self.if_slave_ready_move = True
                    elif order == "lost":
                        counter = 0
                        self.if_finish_move = True
                        return #直接退出return
            else:
                counter += 1
        elif self.current_state == ADJUST:
            pass
        elif self.current_state == MOVE:
            self.my_photo.reset_photo()
            self.if_finish_move = True
            self.my_plan.move_state = NAVIGATE
            self.current_state = NAVIGATE
            self.my_plan.fitting_path_ = []
            return
        elif self.current_state == SERVO:
            self.my_order_manager.clear_knock()
            if self.vision_manager.if_lost_object:
                self.if_finish_move = True
                return
            if self.if_to_the_top:
                self.vision_manager.if_finish_servo = False
                self.vision_manager.if_finish_orbit=True#直接跳过旋转
                self.vision_manager.reset_orbit_angle()
                self.current_state = ORBIT
            elif self.if_first_orbit:#若是第一个环绕
                self.vision_manager.if_finish_servo = False
                self.vision_manager.reset_orbit_angle()
                self.current_state = ORBIT
                self.my_main_protocol.send_start()
            elif self.my_main_protocol.get_slave_state() == "get":#从车完成伺服
                self.vision_manager.if_finish_servo = False
                self.vision_manager.reset_orbit_angle()
                self.current_state = ORBIT
            return
    # 搬运控制函数
    def moving(self):
        if self.if_finish_move:
            return
        if self.current_state == NAVIGATE:
            NAV_T=self.navigate_buffer
            if not self.run_first and not self.get_slave_navigate_state:
                if self.if_send_navigate_command == False:self.if_send_navigate_command = True
                if self.if_send_orbit_command == False:
                    self.if_send_orbit_command = True
                    self.my_main_protocol.send_path(NAV_T['SLA_P'][0],NAV_T['ANGLE'][1],NAV_T['SLA_P'][1])
                if self.my_main_protocol.get_slave_state() == "ready":
                    self.get_slave_navigate_state = True
            else:
                if self.if_first_navigate:
                    self.my_plan.navigate(NAV_T['MAIN_P'],NAV_T['ANGLE'][0],if_high_angle=False,if_first_turn=True)
                else:
                    if self.if_change_side:
                        self.my_plan.navigate(NAV_T['MAIN_P'], NAV_T['ANGLE'][0],if_high_angle=False,if_first_turn=False)
                    else:
                        self.my_plan.navigate(NAV_T['MAIN_P'], NAV_T['ANGLE'][0],if_high_angle=True,if_first_turn=False)
                if self.run_first:
                    '''若主车先走'''
                    if self.if_first_navigate:
                        '''第一次按照中继点设置delay距离'''
                        if len(self.my_plan.path) >=4:slave_message_delay = self.slave_message_delay + (len(self.my_plan.path)-3)*15
                        else:slave_message_delay = self.slave_message_delay
                    elif self.if_delay_more and len(self.my_plan.path) >=4:
                        slave_message_delay = self.slave_message_delay + (len(self.my_plan.path)-3)*15
                    else:slave_message_delay = self.slave_message_delay
                    if self.if_send_orbit_command == False and self.my_plan.finished_dist >= slave_message_delay:
                        '''未发环绕讯息且到达delay距离时'''
                        self.if_send_orbit_command = True
                        self.my_main_protocol.send_path(NAV_T['SLA_P'][0], NAV_T['ANGLE'][1], NAV_T['SLA_P'][1])
                if (self.my_plan.aimed_point_index == len(self.my_plan.path) - 2) and self.my_plan.rest_dist <= self.start_scan_range:
                    self.state_transition()
                    return
        elif self.current_state == SCAN:
            NAV_T=self.navigate_buffer
            self.my_plan.navigate(NAV_T['MAIN_P'], NAV_T['ANGLE'][0], if_first_turn=False)
            self.state_transition()
        elif self.current_state == ORBIT:
            if self.vision_manager.if_finish_orbit:
                self.state_transition() # 退出当前状态，进入搬运状态
                return
            NAV_T=self.navigate_buffer
            gc.collect()  # 环绕前主动回收，缓解内存不足
            self.vision_manager.orbit_control(NAV_T['ANGLE'][2])
        elif self.current_state == ADJUST:
            self.vision_manager.visual_servo_control()
            if self.vision_manager.if_finish_servo == True:
                self.state_transition()
        elif self.current_state == MOVE:
            self.my_photo.update_photo_state()
            if self.my_photo.current_state == OutLine:
                self.reset_car_pos()
                self.my_photo.reset_photo()
                self.my_beep.test()
                self.my_plan.if_finish_navigate = True
            self.my_plan.navigate(path = self.plan_path)
            if self.my_plan.if_finish_navigate == True:
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
                    self.vision_manager.ready_servo_and_orbit(target_point, 'servo')
                    self.my_plan.reset_navigate()
                    self.vision_manager.if_lost_object = False
            
                
