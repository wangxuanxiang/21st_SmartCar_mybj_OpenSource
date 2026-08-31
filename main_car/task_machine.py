'''
# 双车版的任务执行机
def collaborative_task_machine():
    global counter, if_send_preparing_path
    if my_state.state_work == DOWN:
        if my_state.state == my_state.NAVIGATE:
            if if_send_preparing_path == False:
                my_main_protocol.send_path(ord('P'), [[15.0, 0.0], [plan_data.fixed_point[5][0], plan_data.fixed_point[5][1]]])
                if_send_preparing_path = True

            my_plan.navigate([[160.0, plan_data.fixed_point[1][1]]], 0.0)
            # my_plan.navigate([[35.0, -15.0]], 0.0)
            if my_plan.finish_navigate == True:
                # 重置标志位
                my_plan.finish_navigate = False
                if my_vision_manager.failed_servo_count >= 2:
                    my_vision_manager.failed_servo_count = 0
                    my_state.state = my_state.NAVIGATE
                    my_state.state_work = UP
                    if_send_preparing_path = False
                else:
                    my_state.state = my_state.SCAN
                    my_order_manager.mode_target()
                    my_art_protocol.send_object_kind('C')
        elif my_state.state == my_state.SCAN:
            my_plan.navigate([plan_data.fixed_point[1], plan_data.fixed_point[3]], 0.0)
            # my_plan.navigate([plan_data.fixed_point[3]], 0.0)
            if my_plan.finish_navigate == False:
                target_point = my_art_protocol.coordinate_receive()
                if target_point and target_point[1] > my_vision_manager.dist_threshold and (target_point[2] == ord('S') or target_point[2] == ord('T') or target_point[2] == ord('B') or target_point[2] == ord('E') or target_point[2] == ord('W')):  
                    my_vision_manager.ready_servo_and_orbit(target_point)
                    my_plan.reset_navigate()
                    my_state.state = my_state.SERVO
                    # 测试
                    my_beep.test()
            else:
                my_plan.finish_navigate = False
                if_send_preparing_path = False
                # 此时矩形下区域已没有物体，控制小车移动到上区域寻找物体
                my_state.state_work = UP
                my_state.state = my_state.NAVIGATE
                # 将openart置为等待模式
                my_order_manager.finish()
        elif my_state.state == my_state.SERVO:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.visual_servo_control()
            else:
                # 若丢失物体则按矩形轨迹行驶寻找物体
                my_plan.navigate([[my_car.x_current+11.0, my_car.y_current], [my_car.x_current+11.0, my_car.y_current-11.0], [my_car.x_current-11.0, my_car.y_current-11.0], [my_car.x_current-11.0, my_car.y_current], [my_car.x_current, my_car.y_current]], my_vision_manager.target_rel_turn_angle)
                target_point = my_art_protocol.coordinate_receive()
                if target_point and target_point[2] == my_vision_manager.current_servo_object and target_point[1] > my_vision_manager.dist_threshold:
                    my_vision_manager.ready_servo_and_orbit(target_point)
                    my_plan.reset_navigate()
                    my_vision_manager.if_lost_object = False

                # 如果小车在寻找物体过程中完成了一个矩形轨迹但仍未找到物体，则认为该边的区域内没有物体，控制小车再次进行扫描
                if my_plan.finish_navigate == True:
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 重回扫描点继续寻找物体
                    # my_plan.return_to_scan_point = True
                    my_state.state = my_state.NAVIGATE

            if my_vision_manager.finish_servo == True:
                if my_plan.if_send_path == False:
                    my_main_protocol.send_path(my_vision_manager.current_servo_object, [[my_car.x_current, plan_data.fixed_point[1][1]-10.0], [my_car.x_current, my_car.y_current-8.0]])
                    my_plan.if_send_path = True

                if my_main_protocol.get_slave_state() == "get":
                    # 重置标志位
                    my_vision_manager.finish_servo = False
                    my_plan.if_send_path = False

                    my_state.state = my_state.ORBIT
                    # 在采集tof数据时固定小车姿态角
                    my_vision_manager.orbit_turn_angle = my_car.now_yaw * 180 / MATH.PI
                    # 将下一次的扫描点置为当前点偏左，控制小车在该区域内寻找物体
                    # plan_data.fixed_point[1][0] = my_car.x_current-10.0
        elif my_state.state == my_state.ORBIT:
            # 延时100ms，等待稳定后再开始环绕
            if counter <= 10:
                counter += 1
            else:
                my_vision_manager.orbit_control(my_vision_manager.orbit_angle)
                if my_vision_manager.finish_orbit == True:
                    order = my_main_protocol.get_slave_state()
                    if order == "finish":
                        counter = 0
                        # 重置从车视觉伺服失败次数
                        my_vision_manager.failed_servo_count = 0
                        my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                        my_state.state = my_state.MOVE
                        # 提前设置小车转向目标角度为当前角度
                        my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
                    elif order == "lost":
                        counter = 0
                        my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                        my_vision_manager.failed_servo_count += 1
                        # my_state.state = my_state.REVERSE_ORBIT
        elif my_state.state == my_state.MOVE:
            # 控制小车夹紧物体，控制主车提前停止
            my_plan.navigate([[my_car.x_current+my_plan.error_x, -20.0]])
            if my_plan.finish_navigate == True:
                counter = 0
                my_plan.finish_navigate = False
                my_vision_manager.car_position = DOWN_LEFT
                my_state.state = my_state.CALIBRATE
        elif my_state.state == my_state.CALIBRATE:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.apriltag_calibrate_control()
            else:
                # 控制小车前后移动寻找apriltag码
                my_plan.navigate([[my_car.x_current-25.0, my_car.y_current], [my_car.x_current-25.0, my_car.y_current-15.0], [my_car.x_current+10.0, my_car.y_current-15.0], [my_car.x_current+10.0, my_car.y_current+15.0], [my_car.x_current, my_car.y_current+15.0]], my_vision_manager.target_rel_turn_angle)

                target_point = my_art_protocol.apriltag_receive()
                if target_point:
                    my_plan.reset_navigate()
                    my_vision_manager.counter = 0
                    my_vision_manager.calibrate_times = 0
                    my_vision_manager.if_lost_object, my_vision_manager.if_gain_calibrate_angle = False, False

                # 若找不到apriltag码但完成了一个来回的移动轨迹，则认为该边的区域内没有apriltag码，控制小车回到扫描点进行扫描
                if my_plan.finish_navigate == True:
                    # 重置标志位
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    my_state.state = my_state.NAVIGATE
                    # 主车给从车发消息让从车完成矫正
                    my_main_protocol.send_start()
            if my_vision_manager.if_finish_calibrate == True:
                my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                my_state.state = my_state.NAVIGATE
                # 主车给从车发消息让从车完成矫正
                my_main_protocol.send_start()
        # 让小车通过反向环绕恢复原位
        """
        elif my_state.state == my_state.REVERSE_ORBIT:
            my_vision_manager.orbit_control(-my_vision_manager.orbit_angle)
            if my_vision_manager.finish_orbit == True:
                my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                # 重回扫描点继续寻找物体
                my_plan.return_to_scan_point = True
                my_state.state = my_state.NAVIGATE
        """
    elif my_state.state_work == UP:
        if my_state.state == my_state.NAVIGATE:
            if if_send_preparing_path == False:
                # 操控从车从矩形区域左边沿行驶
                my_main_protocol.send_path(ord('P'), [[125.0, 220.0], [plan_data.fixed_point[6][0], plan_data.fixed_point[6][1]]])
                # 之后不用再重置该标志位
                if_send_preparing_path = True
                
            # my_plan.navigate([plan_data.fixed_point[2]], 180.0)
            my_plan.navigate([[160.0, plan_data.fixed_point[2][1]]], 180.0)
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                if my_vision_manager.failed_servo_count >= 2:
                    my_vision_manager.failed_servo_count = 0
                    my_state.state_work = CHECK
                    my_state.state = my_state.NAVIGATE
                else:
                    my_state.state = my_state.SCAN
                    my_vision_manager.my_order_manager.mode_target()
                    my_art_protocol.send_object_kind('C')
        elif my_state.state == my_state.SCAN:
                # my_plan.navigate([plan_data.fixed_point[4]], 180.0)
                my_plan.navigate([plan_data.fixed_point[2], plan_data.fixed_point[4]], 180.0)
                if my_plan.finish_navigate == False:
                    target_point = my_art_protocol.coordinate_receive()
                    if target_point and target_point[1] > my_vision_manager.dist_threshold and (target_point[2] == ord('S') or target_point[2] == ord('T') or target_point[2] == ord('B') or target_point[2] == ord('E') or target_point[2] == ord('W')):
                        my_vision_manager.ready_servo_and_orbit(target_point)
                        my_plan.reset_navigate()
                        my_state.state = my_state.SERVO
                        # 测试
                        my_beep.test()
                else:
                    # 此时矩形上区域已没有物体，控制小车检查区域内是否还有物体遗漏
                    my_plan.finish_navigate = False
                    my_state.if_move_easy_object = True
                    my_state.state_work = CHECK
                    my_state.state = my_state.NAVIGATE
                    my_order_manager.finish()
        elif my_state.state == my_state.SERVO:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.visual_servo_control()
            else:
                # 若丢失物体则按矩形轨迹行驶寻找物体
                my_plan.navigate([[my_car.x_current-11.0, my_car.y_current], [my_car.x_current-11.0, my_car.y_current+11.0], [my_car.x_current+11.0, my_car.y_current+11.0], [my_car.x_current+11.0, my_car.y_current], [my_car.x_current, my_car.y_current]], my_vision_manager.target_rel_turn_angle)
                target_point = my_art_protocol.coordinate_receive()
                if target_point and target_point[2] == my_vision_manager.current_servo_object and target_point[1] > my_vision_manager.dist_threshold:
                    my_vision_manager.ready_servo_and_orbit(target_point)
                    my_plan.reset_navigate()
                    my_vision_manager.if_lost_object = False

                # 如果小车在寻找物体过程中完成了一个矩形轨迹但仍未找到物体，则认为该边的区域内没有物体，控制小车再次进行扫描
                if my_plan.finish_navigate == True:
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 重回扫描点继续寻找物体
                    # my_plan.return_to_scan_point = True
                    my_state.state = my_state.NAVIGATE

            if my_vision_manager.finish_servo == True:
                if my_plan.if_send_path == False:
                    my_main_protocol.send_path(my_vision_manager.current_servo_object, [[my_car.x_current, plan_data.fixed_point[2][1]+10.0], [my_car.x_current, my_car.y_current+8.0]])
                    my_plan.if_send_path = True

                if my_main_protocol.get_slave_state() == "get":
                    # 重置标志位
                    my_vision_manager.finish_servo = False
                    my_plan.if_send_path = False

                    my_state.state = my_state.ORBIT
                    # 在采集tof数据时固定小车姿态角
                    my_vision_manager.orbit_turn_angle = my_car.now_yaw * 180 / MATH.PI
                    # 将下一次的扫描点置为当前点偏右，控制小车在该区域内寻找物体
                    # plan_data.fixed_point[2][0] = my_car.x_current+10.0
        elif my_state.state == my_state.ORBIT:
            # 延时100ms，等待视觉伺服稳定后再开始环绕
            if counter <= 10:
                counter += 1
            else:
                my_vision_manager.orbit_control(my_vision_manager.orbit_angle)
                if my_vision_manager.finish_orbit == True:
                    order = my_main_protocol.get_slave_state()
                    if order == "finish":
                        counter = 0
                        # 重置从车视觉伺服失败次数
                        my_vision_manager.failed_servo_count = 0
                        my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                        my_state.state = my_state.MOVE
                        # 提前设置小车转向目标角度为当前角度
                        my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
                    elif order == "lost":
                        counter = 0
                        my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                        my_vision_manager.failed_servo_count += 1
                        # my_state.state = my_state.REVERSE_ORBIT
        elif my_state.state == my_state.MOVE:
            # 控制小车夹紧物体，控制主车提前停止
            my_plan.navigate([[my_car.x_current-my_plan.error_x, 260.0]])
            if my_plan.finish_navigate == True:
                counter = 0
                my_plan.finish_navigate = False
                my_vision_manager.car_position = UP_RIGHT
                my_state.state = my_state.CALIBRATE
        elif my_state.state == my_state.CALIBRATE:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.apriltag_calibrate_control()
            else:
                # 控制小车前后移动寻找apriltag码
                my_plan.navigate([[my_car.x_current+25.0, my_car.y_current], [my_car.x_current+25.0, my_car.y_current+15.0], [my_car.x_current-10.0, my_car.y_current+15.0], [my_car.x_current-10.0, my_car.y_current-15.0], [my_car.x_current, my_car.y_current-15.0]], -90)

                target_point = my_art_protocol.apriltag_receive()
                if target_point:
                    my_plan.reset_navigate()
                    my_vision_manager.counter = 0
                    my_vision_manager.calibrate_times = 0
                    my_vision_manager.if_lost_object, my_vision_manager.if_gain_calibrate_angle = False, False

                # 若找不到apriltag码但完成了一个来回的移动轨迹，则认为该边的区域内没有apriltag码，控制小车回到扫描点进行扫描
                if my_plan.finish_navigate == True:
                    # 重置标志位
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    my_state.state = my_state.NAVIGATE
                    # 主车给从车发消息让从车完成矫正
                    my_main_protocol.send_start()
            if my_vision_manager.if_finish_calibrate == True:
                # 主车完成矫正后给从车发消息让从车完成矫正
                my_main_protocol.send_start()
                my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                if my_state.if_move_easy_object == False:
                    my_state.state = my_state.NAVIGATE
                else:
                    my_state.state_work = CHECK
                    my_state.state = my_state.NAVIGATE
        # 让小车通过反向环绕恢复原位
        """
        elif my_state.state == my_state.REVERSE_ORBIT:
            my_vision_manager.orbit_control(-my_vision_manager.orbit_angle)
            if my_vision_manager.finish_orbit == True:
                my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                # 重回扫描点继续寻找物体
                my_plan.return_to_scan_point = True
                my_state.state = my_state.NAVIGATE
        """    
    elif my_state.state_work == CHECK:
        if my_state.state == my_state.NAVIGATE:
            my_plan.navigate([plan_data.fixed_point[4]], 180.0)
            if my_plan.finish_navigate == True:
                # 提前让从车到目标点等候
                my_main_protocol.send_path(ord('P'), [[137.0, 240.0]])
                my_plan.finish_navigate = False
                my_state.state = my_state.SCAN
                my_order_manager.mode_target()
                my_art_protocol.send_object_kind('C')
        elif my_state.state == my_state.SCAN:
            my_plan.navigate([[110.0, 140.0], [210.0, 140.0]], 180.0)
            if my_plan.finish_navigate == False:
                target_point = my_art_protocol.coordinate_receive()
                if target_point and (target_point[2] == ord('S') or target_point[2] == ord('T') or target_point[2] == ord('B') or target_point[2] == ord('E') or target_point[2] == ord('W')):
                    my_vision_manager.ready_servo_and_orbit(target_point)
                    my_plan.reset_navigate()
                    my_state.state_work = UP
                    my_state.state = my_state.SERVO
            else:
                if my_plan.if_send_path == False:
                    my_main_protocol.send_path(ord('P'), [[15.0, -15.0]])
                    my_plan.if_send_path = True

                if my_main_protocol.get_slave_state() == "get":
                    my_plan.if_send_path = False
                    my_plan.finish_navigate = False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 此时矩形区域内已没有物体，控制小车返回发车区
                    my_state.state_work = RETURN_WORK
                    my_state.state = my_state.RETURN
    elif my_state.state_work == RETURN_WORK:
        if my_state.state == my_state.RETURN:
            # 最终返回主车的起点（避免回程途中与从车碰撞）
            my_plan.navigate([[25.0, -40.0]], 180.0)
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                my_state.state = my_state.STOP
                my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
        elif my_state.state == my_state.STOP:
            my_plan.stop()


    # 计算环绕中心坐标函数（传入物体中心像素点坐标）
    def calculate_orbit_center(self, x, y):
        raw_x, raw_y = self.pixel_to_real_world(x, y, 'close')
        raw_y = raw_y + self.car_radius # 将物体距离修正为从小车中心到物体的距离
        raw_yaw = -math.atan2(-raw_x, raw_y)
        real_yaw = (raw_yaw + self.my_car.now_yaw + PI) % (2 * PI) - PI
        actual_dist = math.sqrt(raw_x**2 + raw_y**2)
        self.orbit_center_x = self.my_car.x_current + actual_dist * math.sin(real_yaw)
        self.orbit_center_y = self.my_car.y_current + actual_dist * math.cos(real_yaw)

    # 环绕控制函数，传入环绕物体旋转的目标世界坐标系角度（单位：度）（范围：-180到180）
    def orbit_control(self, target_angle: float, direct = None):
        if self.if_orbit_ready == False:
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and target_point[2] == self.current_servo_object and \
            (target_point[0] > 40 or target_point[0] < 120) and target_point[1] > self.dist_threshold:
                self.my_car.alpha_x = 1.0
                self.my_car.alpha_y = 1.0
                # 保持静止
                self.orbit_speed = 0.0
                self.orbit_radius = self.object_radius
                self.record_angle = self.my_car.now_yaw * 180 / PI
                self.target_angle = target_angle
                # 限制目标角度在-180到180度之间
                if self.target_angle > 180.0:
                    self.target_angle -= 360.0
                elif self.target_angle < -180.0:
                    self.target_angle += 360.0
                    
                # 计算需要旋转的相对角度来确定方向
                diff_angle = self.target_angle - self.record_angle
                if diff_angle > 180.0:
                    diff_angle -= 360.0
                elif diff_angle < -180.0:
                    diff_angle += 360.0
                    
                # 确定旋转方向（顺时针还是逆时针）
                if direct is not None:
                    self.direct = direct
                elif diff_angle >= 0.0:
                    self.direct = 'CW'
                else:
                    self.direct = 'CCW'
                self.current_dis = 0.0

                self.calculate_orbit_center(target_point[0], target_point[1])

                self.if_orbit_ready = True
        else:
            if self.if_finish_orbit == True:
                return # 已经完成环绕控制，直接返回
            target_point = self.my_art_protocol.coordinate_receive()

            if target_point and target_point[2] == self.current_servo_object and \
            (target_point[0] > 40 or target_point[0] < 120) and target_point[1] > self.dist_threshold:
                self.calculate_orbit_center(target_point[0], target_point[1])
            
            # ====== 修改：基于当前X/Y坐标的闭环位置控制 ======
            # 计算当前小车与圆心的实际向量
            dx = self.orbit_center_x - self.my_car.x_current
            dy = self.orbit_center_y - self.my_car.y_current
            actual_r = math.sqrt(dx**2 + dy**2)
            # 测试
            # self.my_uart3.write(f"center: ({self.orbit_center_x:.2f}, {self.orbit_center_y:.2f}), car: ({self.my_car.x_current:.2f}, {self.my_car.y_current:.2f})\r\n")
            self.my_uart3.write(f"dx: {dx:.2f}, dy: {dy:.2f}, orbit_r: {self.orbit_radius:.2f}, actual_r: {actual_r:.2f}\r\n")
           
            # 计算当前处于圆上的相位角 (从小车指向圆心)
            theta = -math.atan2(-dx, dy) * 180.0 / PI
            
            # 半径误差（大于0代表实际比指定半径近，需要向外扩）
            err_r = self.orbit_radius - actual_r
           
            # 向心/离心纠正比例 (将厘米级的偏离对应成航向角偏置)
            kr = 10.0
            
            if self.direct == 'CW':
                # 顺时针切线为 theta - 90。若太近(err_r>0)，需向外偏，减小转角
                self.orbit_yaw = theta - 90.0 - kr * err_r
            elif self.direct == 'CCW':
                # 逆时针切线为 theta + 90。若太近(err_r>0)，需向外偏，增加转角
                self.orbit_yaw = theta + 90.0 + kr * err_r
                
            self.orbit_yaw = (self.orbit_yaw + 180.0) % 360.0 - 180.0
            
            # ====== 新增：实时闭环车体姿态角 ======
            # theta 是从圆心指向小车的角度，小车要面向圆心，所以车头朝向应为 theta + 180 度
            self.orbit_turn_angle = theta
            self.orbit_turn_angle = (self.orbit_turn_angle + 180.0) % 360.0 - 180.0
            
            # 更新当前小车的速度（保留原有逻辑判断）
            diff = abs(self.target_angle - self.my_car.now_yaw * 180 / PI)
            if diff > 180.0:
                diff = 360.0 - diff

            # 环绕速度规划：当小车与目标角度的差值大于30度时，保持最大速度；当差值小于30度时，线性降低速度，直到差值小于等于1度时停止
        
            if diff < 40.0:
                self.orbit_speed = self.orbit_v_max - (self.orbit_v_max - self.orbit_v_min) * (40.0 - diff) / 40.0
            else:
                self.orbit_speed = self.orbit_v_max
            # 测试
            # 速度限幅
            self.orbit_speed = max(self.orbit_v_min, min(self.orbit_speed, self.orbit_v_max))

            # 判断是否完成环绕
            # 测试（让小车一直环绕）
            if diff <= 1.0:	
                self.orbit_speed = 0.0
                self.orbit_turn_angle = self.my_car.now_yaw * 180 / PI
                self.my_order_manager.finish()
                self.if_finish_orbit = True


    # 视觉伺服
    # my_uart3.write(f"{servo_pid.actual_x},{servo_pid.target_x},{servo_pid.pwm_output_x},{servo_pid.current_y},{servo_pid.target_y},{servo_pid.pwm_output_y},{my_vision_manager.target_rel_yaw}\n")
    # my_uart3.write(f"object: {my_vision_manager.current_servo_object}, servo_pid.target_y: {servo_pid.target_y}, object_radius: {my_vision_manager.object_radius}, orbit_angle: {my_vision_manager.orbit_angle}\n")
    # my_uart3.write(f"{my_vision_manager.target_rel_speed},{my_vision_manager.target_rel_yaw},{my_vision_manager.target_rel_turn_angle},{my_car.now_yaw * 180 / MATH.PI}\n")
    # my_uart3.write(f"{servo_pid.actual_x},{servo_pid.target_x},{servo_pid.pwm_output_x},{servo_pid.actual_y},{servo_pid.target_y},{servo_pid.pwm_output_y},{my_vision_manager.target_rel_yaw},{my_vision_manager.target_rel_turn_angle},{my_car.now_yaw * 180 / MATH.PI}\n")
    # my_uart3.write(f"{my_vision_manager.my_car.x_current - my_vision_manager.last_car_x},{my_vision_manager.my_car.x_current - my_vision_manager.last_car_x}\r\n")
    # my_uart3.write(f"{my_vision_manager.absolute_actual_x},{my_vision_manager.absolute_actual_y}\n")
    # my_uart3.write(f"{my_vision_manager.real_servo_point}\r\n")

    # 速度环输出波形图调参
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ul_pid.pwm_output, motor_ul_pid.derivative * motor_ul_pid.kd, motor_ul_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ur_pid.target, motor_ur_pid.actual, motor_ur_pid.pwm_output, motor_ur_pid.derivative * motor_ur_pid.kd, motor_ur_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_md_pid.target, motor_md_pid.actual, motor_md_pid.pwm_output, motor_md_pid.derivative * motor_md_pid.kd, motor_md_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ur_pid.target, motor_ur_pid.actual,motor_md_pid.target, motor_md_pid.actual, my_plan.target_v))

    # 路径规划
    # my_uart3.write(f"{my_plan.current_path}\n")	
    
    # 航向角输出
    # my_uart3.write(f"{my_plan.target_yaw}\n")
    # 角度环输出
    # my_uart3.write(f"{angle_pid.kp},{angle_pid.target},{angle_pid.actual},{angle_pid.pwm_output}\n")
    # imu原始数据
    # my_uart3.write("acc = {:>6d}, {:>6d}, {:>6d}\n".format(pose_data.imu_data[0], pose_data.imu_data[1], pose_data.imu_data[2]))
    # my_uart3.write("gyro = {:>6d}, {:>6d}, {:>6d}\n".format(pose_data.imu_data[3], pose_data.imu_data[4], pose_data.imu_data[5]))
                                                                        
    # 里程计：
    # my_uart3.write("ul: {:<f}, ur: {:<f}, md: {:<f}\n".format(my_car.encouder_ul, my_car.encouder_ur, my_car.encouder_md))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current, my_plan.rest_dist, my_plan.target_v, my_car.now_yaw * 180 / MATH.PI))
    
    # 速度规划
    # my_uart3.write(f"{my_plan.waypoint_v}\n")
    # my_uart3.write(f"{my_plan.target_v},{my_plan.target_yaw},{my_plan.aimed_point_index},{my_plan.rest_dist}\n")

    # 检测四元数解算结果是否准确
    # my_uart3.write(f"{pose_data.now_pitch},{pose_data.now_roll},{pose_data.now_yaw},{pose_data.gyro_z}\n")

    # 检测自转角是否准确
    # my_uart3.write("{:<f}\n".format(my_car.now_yaw * 180 / MATH.PI))
    # my_uart3.write(f"{pose_data.gyro_x},{pose_data.gyro_x_bias},{pose_data.gyro_y},{pose_data.gyro_y_bias},{pose_data.gyro_z},{pose_data.gyro_z_bias}\n")
    # 检测gkd项数量级
    # my_uart3.write(f"{pose_data.gyro_z},{pose_data.gyro_z_bias},{pose_data.gyro_z * my_car.gkd}\n")
    
    # apriltag校准测试
    # my_uart3.write(f"{my_vision_manager.angle_temp}\n")
     
    # 环绕测试
    # my_uart3.write(f"{my_vision_manager.orbit_turn_angle}\n")
    # my_uart3.write(f"x_y: {my_vision_manager.orbit_center_x},{my_vision_manager.orbit_center_y}\n")
    # my_uart3.write(f"{my_vision_manager.orbit_yaw},{my_vision_manager.orbit_turn_angle}\n")
    # 搬运控制
    # my_uart3.write(f"state:{my_moving.current_state},{my_plan.if_finish_navigate},{my_vision_manager.if_send_order}\r\n")

    # 任务机
    # my_uart3.write(f"state_work: {my_state.state_work}, state: {my_state.state}, yaw: {my_car.now_yaw * 180 / MATH.PI}, current_object: {my_vision_manager.current_servo_object}, {my_plan.turn_angle_target}\n")

# 视觉伺服测试函数
def test_vision_servo():
    if my_state.state == READY_NAVIGATE:
        if my_vision_manager.if_send_order == False:
            my_order_manager.mode_target()
            my_vision_manager.if_send_order = True

        target_point = my_art_protocol.coordinate_receive()
        if target_point:
            my_vision_manager.ready_servo_and_orbit(target_point)
            # my_vision_manager.calculate_dist(target_point[0], target_point[1], 'far')
            my_vision_manager.if_send_order = False
            my_state.state = SERVO
    elif my_state.state == SERVO:
        my_vision_manager.visual_servo_control()
        if my_vision_manager.if_finish_servo == True:
            my_order_manager.mode_target()
            my_state.state = ORBIT
            my_vision_manager.reset_orbit_angle()
    elif my_state.state == ORBIT:
        my_vision_manager.orbit_control(140.0)
        if my_vision_manager.if_finish_orbit == True:
            my_state.state = STOP
            my_plan.reset_navigate_angle()
    elif my_state.state == STOP:
        my_plan.stop()

# 测试小车搬运状态
def test_moving():
    global counter
    if my_state.state == READY_NAVIGATE:
        my_state.state = NAVIGATE
    elif my_state.state == NAVIGATE:
        my_plan.navigate(path = [[160.0, 80.0]])
        if my_plan.if_finish_navigate == True:
            if my_vision_manager.if_send_order == False:
                my_order_manager.mode_target()
                my_vision_manager.if_send_order = True 
            target_point = my_art_protocol.coordinate_receive()
            if target_point and (target_point[2] in ['T', 'S', 'E', 'W', 'B']):
                my_vision_manager.ready_servo_and_orbit(target_point)
                my_plan.reset_navigate()  # 重置导航相关变量
                my_state.state = SERVO
                my_vision_manager.if_send_order = False 
    elif my_state.state == SERVO:
        my_vision_manager.visual_servo_control()
        if my_vision_manager.if_finish_servo == True:
            counter += 1
            # 延时500ms
            if counter >= 50:
                my_vision_manager.if_finish_servo = False
                my_moving.ready_move()
                my_uart3.write(f"state: {my_moving.current_state},moving_pt: {my_moving.moving_point},angle_buffer: {my_moving.angle_buffer}\n")
                my_order_manager.mode_target()
                my_state.state = MOVE
                # 测试
                my_beep.test()
    elif my_state.state == MOVE:
        my_moving.moving()
        if my_moving.if_finish_move == True:
            my_moving.if_finish_move = False
            my_path.plan_path(plan_data.fixed_point[0][0], plan_data.fixed_point[0][1])
            my_state.state = RETURN
    elif my_state.state == RETURN:
        my_plan.navigate(path = my_path.ready_path, target_turn_angle = 0.0)
        if my_plan.if_finish_navigate == True:
            my_plan.reset_navigate()  # 重置导航相关变量
            my_state.state = STOP
    elif my_state.state == STOP:
        my_plan.stop()

orbit_angle = 240.0
def test_orbit():
    global orbit_angle, counter, direct_flag
    if my_state.state == READY_NAVIGATE:
        my_state.state = ORBIT
        my_vision_manager.object_radius = 16.0
        my_vision_manager.current_servo_object = 'S'
        my_order_manager.mode_target()
    elif my_state.state == ORBIT:
        my_vision_manager.orbit_control(orbit_angle)
        if my_vision_manager.if_finish_orbit == True:
            counter += 1
            if counter >= 100:
                counter = 0
                my_moving.reset_orbit()
                orbit_angle += 120.0
                orbit_angle = (orbit_angle + 180) % 360 - 180

# 边线和apriltag码校准测试程序
def test_apriltag_calibrate():
    global counter
    if my_state.state == READY_NAVIGATE:
        my_state.state = NAVIGATE
    elif my_state.state == NAVIGATE:
        my_plan.navigate(path = [[160.0, 220.0], [0.0, 120.0], [130.0, -5.0]], target_turn_angle = 40.0)
        if my_plan.if_finish_navigate == True:  
            my_plan.reset_navigate()
            my_state.state = CALIBRATE
            my_vision_manager.assist_car_pos = [160.0, 0.0]
            my_vision_manager.car_position = 'R'
    elif my_state.state == CALIBRATE:
        my_vision_manager.apriltag_calibrate_control()
        if my_vision_manager.if_finish_calibrate == True:
            counter += 1
            # 延时1s
            if counter >= 100:
                counter = 0
                my_vision_manager.reset_apriltag_calibrate()
                my_plan.reset_navigate_angle()
                my_state.state = STOP
    elif my_state.state == RETURN:
        my_plan.navigate(path = [[0.0, 0.0]], target_turn_angle = 0.0)
        if my_plan.if_finish_navigate == True:  
            my_plan.reset_navigate()
            my_state.state = STOP
                
spin_angle = 90.0
def test_spin():
    global spin_angle, counter
    if my_state.state == READY_NAVIGATE:
        my_state.state = NAVIGATE
    elif my_state.state == NAVIGATE:
        my_plan.navigate(target_turn_angle = spin_angle)
        if my_plan.if_finish_navigate == True:
            counter += 1
            if counter >= 100:
                counter = 0
                my_plan.reset_navigate()
                spin_angle += 90.0
                spin_angle = (spin_angle + 180) % 360 - 180   

# 调试电机速度环pid函数
def show_speed_PID_test():
    global counter
    counter += 1
    # 调试搬运模式下的pid参数
    # my_state.state = MOVE
    # motor_ul_pid.compute_pid(60, pose_data.encoder_data_ul)
    # motor_ur_pid.compute_pid(300, pose_data.encoder_data_ur)
    # motor_md_pid.compute_pid(300, pose_data.encoder_data_md)

    # 测试不同速度下的pid参数切换情况
    if counter >= 8000:
        counter = 0
    elif counter >= 6000:
        motor_ul_pid.compute_pid(180, pose_data.encoder_data_ul)
    elif counter >= 4000:
        motor_ul_pid.compute_pid(-20, pose_data.encoder_data_ul)
    elif counter >= 2000:
        motor_ul_pid.compute_pid(-120, pose_data.encoder_data_ul)
    else:
        motor_ul_pid.compute_pid(250, pose_data.encoder_data_ul)


'''

        '''
        self.my_plan.navigate(path = self.scan_message)
        target_point = self.my_art_protocol.coordinate_receive()
        if self.if_rogue_plan:
            if target_point:
                self.my_vision.current_servo_object = self.current_object
            if target_point and chr(target_point[2]) == self.current_object and self.my_moving.ready_move(target_point):  # 准备搬运动作  
                self.my_vision.ready_servo_and_orbit(target_point, 'servo')
                self.my_beep.test()  # 扫描到目标物体，发出提示�?
                self.exit()  # 退出当前状态，进入扫描状�?
                return
        else:
            if target_point:
                self.current_object=chr(target_point[2])
                self.my_plan.current_object = self.current_object
                self.my_vision.current_servo_object = self.current_object
            if target_point and self.my_moving.ready_move(target_point) : # 准备搬运动作
                self.my_vision.ready_servo_and_orbit(target_point, 'servo')
                self.my_beep.test()  # scan found target
                self.exit()  # leave scan
                return
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入伺服状�?
            return
        
        def max_block():
            gc.collect()
            low = 0
            high = gc.mem_free()
            while low + 16 < high:
                mid = (low + high) // 2
                try:
                    b = bytearray(mid)
                    del b
                    low = mid
                except MemoryError:
                    high = mid
                gc.collect()
            return low
        def mem(tag):
            print(tag,gc.mem_free(),gc.mem_alloc(),max_block())
            #my_uart3.write(f"{tag},{gc.mem_free()}, {gc.mem_alloc()}, {max_block()}\n")
            gc.collect()
    '''