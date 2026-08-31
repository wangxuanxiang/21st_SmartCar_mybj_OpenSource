# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 从 seekfree 库包含 KEY_HANDLER
from seekfree import KEY_HANDLER

# 包含 gc 与 time 类
import gc
import time

import os

class Menu:
    def __init__(self, flash_sys, beep, lcd, enc_rotation, key_data, key_handler):   
        # 所有参数强制转为浮点数，避免类型错误
        self.config = {
            # PID 参数
            "angle_normal_kp": float(flash_sys.find_value("angle_normal_kp")) if flash_sys.find_value("angle_normal_kp") else 10.0,
            "angle_normal_ki": float(flash_sys.find_value("angle_normal_ki")) if flash_sys.find_value("angle_normal_ki") else 0.0,
            "angle_normal_kd": float(flash_sys.find_value("angle_normal_kd")) if flash_sys.find_value("angle_normal_kd") else 20.0,
            "integral_limitmax": float(flash_sys.find_value("integral_limitmax")) if flash_sys.find_value("integral_limitmax") else 14000.0,
            "pwmout_limitmax": float(flash_sys.find_value("pwmout_limitmax")) if flash_sys.find_value("pwmout_limitmax") else 8000.0,
            "high_angle_pwmout_limitmax": float(flash_sys.find_value("high_angle_pwmout_limitmax")) if flash_sys.find_value("high_angle_pwmout_limitmax") else 600.0,
            "low_angle_pwmout_limitmax": float(flash_sys.find_value("low_angle_pwmout_limitmax")) if flash_sys.find_value("low_angle_pwmout_limitmax") else 200.0,
            "high_angle_deadband": float(flash_sys.find_value("high_angle_deadband")) if flash_sys.find_value("high_angle_deadband") else 15.0,
            # A/B 设置
            "A": float(flash_sys.find_value("A")) if flash_sys.find_value("A") else 500.0,
            "B": float(flash_sys.find_value("B")) if flash_sys.find_value("B") else 200.0,
            # 系数
            "gkd": float(flash_sys.find_value("gkd")) if flash_sys.find_value("gkd") else -0.19,
            "speed_fuse_ratio": float(flash_sys.find_value("speed_fuse_ratio")) if flash_sys.find_value("speed_fuse_ratio") else 0.2,
            # 时间规划
            "motor_control_T": float(flash_sys.find_value("motor_control_T")) if flash_sys.find_value("motor_control_T") else 2.0,
            "collect_dt": float(flash_sys.find_value("collect_dt")) if flash_sys.find_value("collect_dt") else 0.002,
            "plan_calculate_T": float(flash_sys.find_value("plan_calculate_T")) if flash_sys.find_value("plan_calculate_T") else 10.0,
            "uart_and_menu_T": float(flash_sys.find_value("uart_and_menu_T")) if flash_sys.find_value("uart_and_menu_T") else 53.0,
            "boost_time_threshold": float(flash_sys.find_value("boost_time_threshold")) if flash_sys.find_value("boost_time_threshold") else 60.0,
            # 路径规划
            "plan_arrive_threshold": float(flash_sys.find_value("plan_arrive_threshold")) if flash_sys.find_value("plan_arrive_threshold") else 1.0,
            "plan_point_transition_T": float(flash_sys.find_value("plan_point_transition_T")) if flash_sys.find_value("plan_point_transition_T") else 50.0,
            # 速度规划
            "min_start_v": float(flash_sys.find_value("min_start_v")) if flash_sys.find_value("min_start_v") else 40.0,
            "long_v_max": float(flash_sys.find_value("long_v_max")) if flash_sys.find_value("long_v_max") else 400.0,
            "short_v_max": float(flash_sys.find_value("short_v_max")) if flash_sys.find_value("short_v_max") else 50.0,
            "dead_zone_v": float(flash_sys.find_value("dead_zone_v")) if flash_sys.find_value("dead_zone_v") else 50.0,
            "transit_v": float(flash_sys.find_value("transit_v")) if flash_sys.find_value("transit_v") else 200.0,
            "orbit_v": float(flash_sys.find_value("orbit_v")) if flash_sys.find_value("orbit_v") else 100.0,
            "move_v_max": float(flash_sys.find_value("move_v_max")) if flash_sys.find_value("move_v_max") else 150.0,
            "scan_v_max": float(flash_sys.find_value("scan_v_max")) if flash_sys.find_value("scan_v_max") else 80.0,
            # 视觉伺服
            "servo_kp_x": float(flash_sys.find_value("servo_kp_x")) if flash_sys.find_value("servo_kp_x") else 8.0,
            "servo_kd_x": float(flash_sys.find_value("servo_kd_x")) if flash_sys.find_value("servo_kd_x") else 10.0,
            "servo_kp_y": float(flash_sys.find_value("servo_kp_y")) if flash_sys.find_value("servo_kp_y") else 8.0,
            "servo_kd_y": float(flash_sys.find_value("servo_kd_y")) if flash_sys.find_value("servo_kd_y") else 10.0,
            "servo_target_x": float(flash_sys.find_value("servo_target_x")) if flash_sys.find_value("servo_target_x") else 80.0,
            "servo_target_y_T": float(flash_sys.find_value("servo_target_y_T")) if flash_sys.find_value("servo_target_y_T") else 5.0,
            "servo_target_y_S": float(flash_sys.find_value("servo_target_y_S")) if flash_sys.find_value("servo_target_y_S") else 8.0,
            "servo_target_y_B": float(flash_sys.find_value("servo_target_y_B")) if flash_sys.find_value("servo_target_y_B") else 30.0,
            "min_rel_speed": float(flash_sys.find_value("min_rel_speed")) if flash_sys.find_value("min_rel_speed") else 50.0,
            "max_rel_speed": float(flash_sys.find_value("max_rel_speed")) if flash_sys.find_value("max_rel_speed") else 250.0,
            "finish_threshold_x": float(flash_sys.find_value("finish_threshold_x")) if flash_sys.find_value("finish_threshold_x") else 2.0,
            "finish_threshold_y": float(flash_sys.find_value("finish_threshold_y")) if flash_sys.find_value("finish_threshold_y") else 4.0,
            "servo_pwmout_limitmax": float(flash_sys.find_value("servo_pwmout_limitmax")) if flash_sys.find_value("servo_pwmout_limitmax") else 250.0,
            # 环绕控制
            "radius_T": float(flash_sys.find_value("radius_T")) if flash_sys.find_value("radius_T") else 2.5,
            "radius_S": float(flash_sys.find_value("radius_S")) if flash_sys.find_value("radius_S") else 4.5,
            "radius_B": float(flash_sys.find_value("radius_B")) if flash_sys.find_value("radius_B") else 13.0,
        } # 用字典保存所需改的参数

        # 参数名-缩略名映射字典
        self.param_short_name = {
            # PID 参数
            "angle_normal_kp": "n_kp",
            "angle_normal_ki": "n_ki",
            "angle_normal_kd": "n_kd",
            "integral_limitmax": "int_l",
            "pwmout_limitmax": "pwm_l",
            "high_angle_pwmout_limitmax": "h_a_pwm",
            "low_angle_pwmout_limitmax": "l_a_pwm",
            "A": "A",
            "B": "B",
            # 系数
            "gkd": "gkd",
            "speed_fuse_ratio": "fuse_rat",
            # 时间规划
            "motor_control_T": "motor_T",
            "collect_dt": "col_dt",
            "plan_calculate_T": "plan_T",
            "uart_and_menu_T": "uart_T",
            "boost_time_threshold": "boost_T",
            # 路径规划
            "plan_arrive_threshold": "arrive_th",
            "plan_point_transition_T": "trans_T",
            # 速度规划
            "min_start_v": "min_v",
            "long_v_max": "long_v",
            "short_v_max": "short_v",
            "dead_zone_v": "dead_v",
            "transit_v": "trans_v",
            "orbit_v": "orbit_v",
            "move_v_max": "move_v",
            "scan_v_max": "scan_v",
            # 视觉伺服
            "servo_kp_x": "kp_x",
            "servo_kd_x": "kd_x",
            "servo_kp_y": "kp_y",
            "servo_kd_y": "kd_y",
            "servo_target_x": "tar_x",
            "servo_target_y_T": "tar_yT",
            "servo_target_y_S": "tar_yS",
            "servo_target_y_B": "tar_yB",
            "min_rel_speed": "min_spd",
            "max_rel_speed": "max_spd",
            "finish_threshold_x": "fin_x",
            "finish_threshold_y": "fin_y",
            "servo_pwmout_limitmax": "servo_pw",
            # 环绕控制
            "radius_T": "rad_T",
            "radius_S": "rad_S",
            "radius_B": "rad_B",
        }
        gc.collect()
        # 注入外部硬件对象
        self.flash_sys = flash_sys
        self.beep = beep
        self.lcd = lcd
        self.enc_rotation = enc_rotation  # 编码器旋转对象
        self.key_data = key_data            # 编码器按键引脚对象
        self.key_handler = key_handler
        self.key_index_map = {"up":1, "down":0, "confirm":2}

        # 菜单核心配置
        self.change_page_to = 1
        self.Current_line = 2  # 初始箭头在第二行
        self.Start_line, self.End_line = 1, 9
        # 步长
        self.step_values = (0.001, 0.01, 0.1, 1.0, 5.0, 10.0, 100.0)  # 步长（元组更省内存）
        self.current_step_index = 0
        # 行间距
        self.LineSpacing = 18
        
        # 屏幕配置
        self.LCD_WIDTH = 240
        self.TITLE_X = 80  # 240宽度屏标题居中x坐标
        self.ARROW_X = 200  # 箭头x坐标
        self.CLEAR_SPACES = " " * 35

        # 编码器旋转相关状态
        self.enc_rot_debounce_ms = 40                  # 旋转防抖时间
        self.enc_rot_last_trigger_time = time.ticks_ms()
        self.enc_pulse_threshold = 5

        # 编码器按键相关状态
        self.is_param_selected = False # 参数选中状态（初始未选中）
        self.selected_line = None      # 选中的行号
        # 颜色定义
        self.COLOR_WHITE = 0xFFFF     # 白色（默认箭头）
        self.COLOR_RED = 0xF800       # 红色（选中状态箭头）

        # 状态标记：初始设置为True，确保首次绘制箭头
        self.last_change_page_to = self.change_page_to
        self.current_key = None
        self.need_refresh = True
        
        # 预定义核心映射
        self._init_core_mappings()

        # 保证初始显示
        self.menu_switch()  # 绘制默认的PID页
        self.refresh_current_page()  # 绘制初始箭头（第二行）
        
        # 强制GC
        gc.collect()

    def _init_core_mappings(self):
        """初始化核心映射（8页分区）"""
        self.page_line_map = {
            # 1.PID页
            1: {
                2: ("angle_normal_kp", "6.3f"), 
                3: ("angle_normal_ki", "6.3f"), 
                4: ("angle_normal_kd", "6.3f"),
                5: ("integral_limitmax", "6.3f"), 
                6: ("pwmout_limitmax", "6.3f"), 
                7: ("high_angle_pwmout_limitmax", "6.3f"),
                8: ("low_angle_pwmout_limitmax", "6.3f"), 
                9: ("A", "6.3f"), 
                10: ("B", "6.3f")
            },
            # 2.COEF页（系数参数）
            2: {
                2: ("gkd", "6.3f"), 
                3: ("speed_fuse_ratio", "6.3f")
            },
            # 3.TIME页（时间规划参数）
            3: {
                2: ("motor_control_T", "6.3f"), 
                3: ("collect_dt", "6.3f"),
                4: ("plan_calculate_T", "6.3f"),
                5: ("uart_and_menu_T", "6.3f"), 
                6: ("boost_time_threshold", "6.3f")
            },
            # 4.PATH页（路径规划参数）
            4: {
                2: ("plan_arrive_threshold", "6.3f"), 
                3: ("plan_point_transition_T", "6.3f")
            },
            # 5.SPEED页（速度规划参数）
            5: {
                2: ("min_start_v", "6.3f"), 
                3: ("long_v_max", "6.3f"),
                4: ("short_v_max", "6.3f"), 
                5: ("dead_zone_v", "6.3f"),
                6: ("transit_v", "6.3f"), 
                7: ("orbit_v", "6.3f"),
                8: ("move_v_max", "6.3f"), 
                9: ("scan_v_max", "6.3f")
            },
            # 6.SERVO页（视觉伺服参数）
            6: {
                2: ("servo_kp_x", "6.3f"), 
                3: ("servo_kd_x", "6.3f"),
                4: ("servo_kp_y", "6.3f"), 
                5: ("servo_kd_y", "6.3f"),
                6: ("servo_target_x", "6.3f"), 
                7: ("servo_target_y_T", "6.3f"),
                8: ("servo_target_y_S", "6.3f"), 
                9: ("servo_target_y_B", "6.3f"),
                10: ("min_rel_speed", "6.3f"), 
                11: ("max_rel_speed", "6.3f"),
                12: ("finish_threshold_x", "6.3f"), 
                13: ("finish_threshold_y", "6.3f"),
                14: ("servo_pwmout_limitmax", "6.3f")
            },
            # 7.ORBIT页（环绕控制参数）
            7: {
                2: ("radius_T", "6.3f"), 
                3: ("radius_S", "6.3f"),
                4: ("radius_B", "6.3f")
            }
        }

        # 页面-保存行映射（按每页参数数量+1（步长行）计算保存行）
        self.save_line_map = {
            1: 11,   # PID页：9个参数 → 11行保存
            2: 4,    # COEF页：2个参数 → 4行保存
            3: 7,    # TIME页：5个参数 → 7行保存
            4: 4,    # PATH页：2个参数 → 4行保存
            5: 10,   # SPEED页：8个参数 → 10行保存
            6: 15,   # SERVO页：13个参数 →15行保存
            7: 5     # ORBIT页：3个参数 →5行保存
        }

        # 页面配置（保存参数列表）
        self.page_configs = {
            1: ["angle_normal_kp", "angle_normal_ki", "angle_normal_kd", "integral_limitmax",
                "pwmout_limitmax", "high_angle_pwmout_limitmax", "low_angle_pwmout_limitmax", 
                "A", "B"],
            2: ["gkd", "speed_fuse_ratio"],
            3: ["motor_control_T", "collect_dt", "plan_calculate_T", "uart_and_menu_T", "boost_time_threshold"],
            4: ["plan_arrive_threshold", "plan_point_transition_T"],
            5: ["min_start_v", "long_v_max", "short_v_max", "dead_zone_v", "transit_v",
                "orbit_v", "move_v_max", "scan_v_max"],
            6: ["servo_kp_x", "servo_kd_x", "servo_kp_y", "servo_kd_y", "servo_target_x",
                "servo_target_y_T", "servo_target_y_S", "servo_target_y_B", "min_rel_speed",
                "max_rel_speed", "finish_threshold_x", "finish_threshold_y", "servo_pwmout_limitmax"],
            7: ["radius_T", "radius_S", "radius_B"]
        }

        # 页面-标题-行范围映射
        self.page_meta = {
            1: ("PID", 1, 11),    # PID：9参数+步长+保存 → 1-11行
            2: ("COEF", 1, 4),    # COEF：2参数+步长+保存 →1-4行
            3: ("TIME", 1, 7),    # TIME：5参数+步长+保存 →1-7行
            4: ("PATH", 1, 4),    # PATH：2参数+步长+保存 →1-4行
            5: ("SPEED", 1, 10),  # SPEED：8参数+步长+保存 →1-10行
            6: ("SERVO", 1, 15),  # SERVO：13参数+步长+保存 →1-15行
            7: ("ORBIT", 1, 5)    # ORBIT：3参数+步长+保存 →1-5行
        }

    def _is_param_line(self, line_num):
        """判断当前行是否为参数行（非步长/save行）"""
        page_map = self.page_line_map.get(self.change_page_to, {})
        return line_num in page_map.keys()
    
    def _is_save_line(self, line_num):
        return line_num == self.save_line_map.get(self.change_page_to, 0)

    def _format_param_display(self, config_key):
        """通用参数格式化方法（三位小数显示）"""
        short_name = self.param_short_name.get(config_key, config_key[:6])
        val = round(self.config[config_key], 3)  # 保留三位小数
        return f"{short_name:<6} : {val:8.3f}"

    # 局部刷新参数行
    def refresh_param_line(self, line_num, config_key):
        """仅刷新指定行的参数"""
        # 清空当前行
        self.lcd.str16(0, self.LineSpacing * line_num, self.CLEAR_SPACES, 0x0000)
        # 绘制格式化后的参数（完整显示三位小数）
        display_text = self._format_param_display(config_key)
        self.lcd.str16(5, self.LineSpacing * line_num, display_text, 0xFFFF)
        self._redraw_current_arrow()
        gc.collect()

    def _redraw_current_arrow(self):
        """仅重绘当前行的箭头，保持选中状态颜色"""
        arrow_color = self.COLOR_RED if (self.is_param_selected and self.selected_line == self.Current_line) else self.COLOR_WHITE
        self.lcd.str16(self.ARROW_X, self.LineSpacing * self.Current_line, "<--", arrow_color)
    # ========== 读取编码器旋转（left/right） ==========
    def read_encoder_rotation(self):
        """读取编码器旋转方向，返回left/right/None（防抖）"""
        current_time = time.ticks_ms()
        
        # 防抖检查
        if time.ticks_diff(current_time, self.enc_rot_last_trigger_time) < self.enc_rot_debounce_ms:
            return None

        self.enc_rotation.capture()
        current_enc = self.enc_rotation.get()

        if abs(current_enc) >= self.enc_pulse_threshold:
            self.enc_rot_last_trigger_time = current_time

            if current_enc > 0:
                self.beep.key_test()
                return "right"
            elif current_enc < 0:
                self.beep.key_test()
                return "left"
        gc.collect()
        return None

    # ========== 读取编码器按键 ==========
    def read_confirm_key(self):
        """读取确认键"""
        confirm_idx = self.key_index_map["confirm"]
        if self.key_data[confirm_idx] == 1:
            self.key_handler.clear(confirm_idx + 1)
            self.beep.key_test()  # 按键松开时触发蜂鸣
            return True
        gc.collect()
        return False
    

    # ========== 切换参数选中状态 ==========
    def toggle_param_select(self):
        """切换参数选中状态（仅对参数行生效）"""
        # 仅参数行可选中
        if not self._is_param_line(self.Current_line):
            self.is_param_selected = False
            self.selected_line = None
            self.refresh_current_page()  # 刷新为白色箭头
            return
        
        # 切换选中状态
        if self.is_param_selected and self.selected_line == self.Current_line:
            # 取消选中
            self.is_param_selected = False
            self.selected_line = None
        else:
            # 选中当前参数行
            self.is_param_selected = True
            self.selected_line = self.Current_line
        gc.collect()
        # 刷新箭头颜色
        self.refresh_current_page()

    # 刷新当前页面（支持红色箭头）
    def refresh_current_page(self):
        """刷新箭头区域，支持选中状态红色箭头"""
        # 清空所有箭头位置
        for i in range(self.Start_line, self.End_line + 1):
            self.lcd.str16(self.ARROW_X, self.LineSpacing * i, "   ", self.COLOR_WHITE)
        
        # 绘制当前箭头（根据选中状态切换颜色）
        arrow_color = self.COLOR_RED if (self.is_param_selected and self.selected_line == self.Current_line) else self.COLOR_WHITE
        self.lcd.str16(self.ARROW_X, self.LineSpacing * self.Current_line, "<--", arrow_color)
        
        self.need_refresh = False
        gc.collect()

    # 显式销毁方法（释放所有外部对象引用）
    def destroy(self):
        """销毁实例，释放内存"""
        try:
            # 修复：只清理存在的属性
            for attr in ['config', 'param_short_name']:
                if hasattr(self, attr) and isinstance(getattr(self, attr), dict):
                    getattr(self, attr).clear()
            
            # 释放所有外部硬件对象引用
            self.flash_sys = None
            self.beep = None
            self.lcd = None
            self.enc_rotation = None
            self.key_handler = None
            self.key_data = None
            
            self.last_change_page_to = None
            self.current_key = None
            self.is_param_selected = False
            self.selected_line = None
            self.need_refresh = False
            gc.collect()
        except Exception as e:
            print(f"Destroy error: {e}")
        gc.collect()

    # 批量更新配置（保存三位小数）
    def update_config_values(self, file_path, updates):
        """安全更新配置文件（临时文件+原子替换）"""
        temp_file_path = file_path + ".tmp"
        try:
            with open(file_path, 'r') as f_in, open(temp_file_path, 'w') as f_out:
                for line in f_in:
                    stripped = line.strip()
                    if not stripped or stripped.startswith('#'):
                        f_out.write(line)
                        continue
                    if '=' in stripped:
                        key, _ = stripped.split('=', 1)
                        key = key.strip()
                        if key in updates:
                            f_out.write(f"{key} = {updates[key]:.3f}\n")  # 保存为三位小数
                        else:
                            f_out.write(line)
                    else:
                        f_out.write(line)
            gc.collect()
            os.remove(file_path)
            os.rename(temp_file_path, file_path)
        except Exception as e:
            if os.path.exists(temp_file_path):
                os.remove(temp_file_path)
            print(f"Config update error: {e}")
        finally:
            gc.collect()

    # 统一保存数据（保存三位小数）
    def save_data(self):
        """保存当前页面参数（三位小数）"""
        updates = {k: round(self.config[k], 3) for k in self.page_configs.get(self.change_page_to, [])}
        if updates:
            self.update_config_values(self.flash_sys.file_path, updates)
        gc.collect()
        self.Current_line = 2
        self.is_param_selected = False  # 保存后取消选中
        self.selected_line = None
        self.need_refresh = True
        gc.collect()

    # 读取按键（整合所有输入：上下键+编码器旋转+编码器按键）
    def read_key(self):
        """读取所有输入"""
        pressed_key = None
    
        # 读取编码器按键（返回特殊标识）
        if self.read_confirm_key():
            return "confirm"
        
        # 读取up/down按键
        if self.key_data[1] == 1:
            self.key_handler.clear(2)
            self.beep.key_test()
            pressed_key = "up"
        elif self.key_data[0] == 1:
            self.key_handler.clear(1)
            self.beep.key_test()
            pressed_key = "down"

        if pressed_key is None:
            enc_rot_key = self.read_encoder_rotation()
            if enc_rot_key:
                pressed_key = enc_rot_key
        gc.collect()
        return pressed_key
        
    # 数据处理（增加选中状态判断）
    def data_processing(self, key):
        """处理参数修改逻辑（仅选中状态可修改参数）"""
        # 步长行：无需选中即可修改
        if self.Current_line == 1:
            old_index = self.current_step_index
            if key == "left":
                self.current_step_index = (self.current_step_index - 1) % len(self.step_values)
            elif key == "right":
                self.current_step_index = (self.current_step_index + 1) % len(self.step_values)
            gc.collect()
            if old_index != self.current_step_index:
                self.lcd.str16(0, self.LineSpacing * 1, self.CLEAR_SPACES, 0x0000)
                step_text = f"step : {self.step_values[self.current_step_index]:8.3f}"
                self.lcd.str16(5, self.LineSpacing * 1, step_text, 0xFFFF)
                self._redraw_current_arrow()
            gc.collect()
            return

        # 参数行：仅选中状态可修改
        line_config = self.page_line_map.get(self.change_page_to, {}).get(self.Current_line)
        if line_config and self.is_param_selected and self.selected_line == self.Current_line:
            config_key, _ = line_config
            step = self.step_values[self.current_step_index]
            
            if key == "left":
                self.config[config_key] = round(self.config[config_key] - step, 3)
            elif key == "right":
                self.config[config_key] = round(self.config[config_key] + step, 3)
            
            self.refresh_param_line(self.Current_line, config_key)
            gc.collect()
            return

        # save行：无需选中即可保存
        if self.Current_line == self.save_line_map.get(self.change_page_to, 0) and key == "confirm":
            self.save_data()
        
        gc.collect()

    # 箭头控制（移动箭头时取消选中）
    def move_arrow(self, key):
        """移动箭头位置（移动时取消参数选中）"""
        # 清空当前箭头
        self.lcd.str16(self.ARROW_X, self.LineSpacing * self.Current_line, "   ", self.COLOR_WHITE)
        # 移动箭头
        if key == "up":
            self.Current_line = self.End_line if self.Current_line <= self.Start_line else self.Current_line - 1
        elif key == "down":
            self.Current_line = self.Start_line if self.Current_line >= self.End_line else self.Current_line + 1
        # 移动箭头时取消选中状态
        self.is_param_selected = False
        self.selected_line = None
        # 绘制新箭头（白色）
        self._redraw_current_arrow()
        self.need_refresh = True
        gc.collect()

    # 页面切换检测（适配7页翻页逻辑）
    def detect_change_page(self, key):
        """检测并处理7页页面切换"""
        is_save_line = self.Current_line == self.save_line_map.get(self.change_page_to, 0)
        if (self.is_param_selected is False and self.Current_line != 1) or is_save_line:
            if key == "left":
                self.change_page_to = 7 if self.change_page_to == 1 else self.change_page_to - 1
            elif key == "right":
                self.change_page_to = 1 if self.change_page_to == 7 else self.change_page_to + 1
            
            self.Current_line = 2
            self.is_param_selected = False  # 翻页取消选中
            self.selected_line = None
            self.last_change_page_to = self.change_page_to
            self.lcd.clear(0x0000)
            self.menu_switch()
            self.refresh_current_page()
            self.need_refresh = True
            gc.collect()
            return True
        return False

    # 页面显示
    def _show_page(self, page_num):
        """通用页面绘制方法（无额外文本，适配三位小数）"""
        title, start_line, end_line = self.page_meta.get(page_num, ("", 1, 14))
        self.Start_line, self.End_line = start_line, end_line
        
        # 清空标题行和步长行
        self.lcd.str16(0, 0, self.CLEAR_SPACES, 0x0000)
        self.lcd.str16(0, self.LineSpacing * 1, self.CLEAR_SPACES, 0x0000)
        
        # 绘制标题
        self.lcd.str16(self.TITLE_X, 0, title, 0xFFFF)
        # 绘制步长
        step_text = f"step : {self.step_values[self.current_step_index]:8.3f}"
        self.lcd.str16(5, self.LineSpacing * 1, step_text, 0xFFFF)
        
        # 绘制参数行
        data_lines = self.page_line_map.get(page_num, {})
        max_line = max(data_lines.keys()) if data_lines else 1
        for line_num in range(2, max_line + 1):
            if line_num in data_lines:
                config_key, _ = data_lines[line_num]
                self.refresh_param_line(line_num, config_key)
        
        # 绘制save行
        save_line = self.save_line_map.get(page_num, 0)
        
        if save_line:
            self.lcd.str16(0, self.LineSpacing * save_line, self.CLEAR_SPACES, 0x0000)
            self.lcd.str16(5, self.LineSpacing * save_line, "save", 0xFFFF)
        
        gc.collect()

    # 页面切换
    def menu_switch(self):
        """切换到指定页面"""
        self._show_page(self.change_page_to)

    # 按键处理入口
    def handle_key_from_interrupt(self, key):
        """按键处理主入口（适配中断）"""
        if not key:
            return
        
        # 处理编码器按下（切换选中状态）
        if key == "confirm":
            if self._is_save_line(self.Current_line):
                self.save_data()
            else:
                self.toggle_param_select()
        # 处理up/down（移动箭头）
        elif key in ("up", "down"):
            self.move_arrow(key)
        # 处理left/right（编码器旋转，仅选中参数行可修改）
        elif key in ("left", "right"):
            if not self.detect_change_page(key):
                self.data_processing(key)
        if self.need_refresh:
            self.refresh_current_page()