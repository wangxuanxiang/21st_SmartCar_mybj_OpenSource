from micropython import const
import math
import gc

_PI = const(3.1415926)
_READY_NAVIGATE = const(0) # 准备导航状态
_NAVIGATE = const(1)       # 导航状态
_SCAN = const(2)           # 扫描状态
_SERVO = const(3)          # 视觉伺服状态
_ORBIT = const(4)          # 环绕状态
_MOVE = const(5)           # 搬运状态
_CALIBRATE = const(6)      # 校准状态
_ADJUST = const(7)         # 微调状态
_RETURN = const(8)		  # 返回状态
_STOP = const(9)           # 停止状态
_RETREAT = const(10)       # 后退状态

# 状态机
class StateMachine:
    def __init__(self):        
        self.if_move_easy_object = False   # 是否搬运过易搬运物体的标志位（搬运过易搬运物体后在返回起点时不避开矩形区域）
        self.state = _READY_NAVIGATE  # 初始状态为准备导航状态
        gc.collect()

# 路径和速度规划相关常量
class PlanData:
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 地图固定点坐标
        # fixed_point[0]为从车起点，fixed_point[1]为矩形框左下方顶点，fixed_point[2]为矩形框右上方顶点, fixed_point[3]为从车返回点
        self.center_x = self.flash_sys.find_value("CENTER_X")
        self.center_y = self.flash_sys.find_value("CENTER_Y")
        self.lenth = self.flash_sys.find_value("SUDOKU_length_x")

        rate = self.flash_sys.find_value("rect_zoom_rate")
        rate_vision = self.flash_sys.find_value("vision_rect_zoom_rate")
        self.fixed_point = [[15.0, -40.0], [self.center_x - self.lenth*rate,self.center_y - self.lenth*rate], 
                            [self.center_x + self.lenth*rate,self.center_y + self.lenth*rate],[25.0, -50.0]]  # type: list
        # 中心物品摆放的矩形区域
        self.center_rect = [[self.center_x - self.lenth*rate,self.center_y - self.lenth*rate], [self.center_x - self.lenth*rate,self.center_y + self.lenth*rate], 
                            [self.center_x + self.lenth*rate,self.center_y - self.lenth*rate], [self.center_x + self.lenth*rate,self.center_y + self.lenth*rate]] 

        # 中心物品摆放的矩形区域（视觉过滤）
        self.vision_fil_center_rect = [[self.center_x - self.lenth*rate_vision,self.center_y - self.lenth*rate_vision], [self.center_x - self.lenth*rate_vision,self.center_y + self.lenth*rate_vision],
                                       [self.center_x + self.lenth*rate_vision,self.center_y - self.lenth*rate_vision], [self.center_x + self.lenth*rate_vision,self.center_y + self.lenth*rate_vision]]
        # 路径规划相关常量
        self.FIELD_W = self.flash_sys.find_value("FIELD_W")  # 地图宽度
        self.FIELD_H = self.flash_sys.find_value("FIELD_H")  # 地图高度
        self.OBSTACLE_R = 16.0  # 圆形障碍物默认半径 (直径 30cm -> 半径 15cm)
        self.CUBE_LENTH = 23.8   # 立方体障碍物长度
        self.CUBE_WIDE = 10.6  # 立方体障碍物宽度
        self.INF = 1000000000.0  # 无穷大
        self.SAFE_MARGIN = self.flash_sys.find_value("SAFE_MARGIN")# 小车安全裕量 (质点膨胀半径)

        # 中心矩形输入时已经膨胀
        self.rectangle_obstacles = self.create_expanded_rect(self.center_x, self.center_y, self.lenth*rate*2, self.lenth*rate*2, if_expand=False)  # 中心禁区矩形障碍物（已膨胀）
        self.if_obstacles = self.flash_sys.find_value("if_obstacles")
        if self.if_obstacles == 1:
            self.cube = self.flash_sys.find_value("cube_obstacles")  # 立方体障碍物中心坐标列表（未膨胀）
            self.circle = self.flash_sys.find_value("circle")  # 信标障碍物中心坐标列表
        else:
            self.cube = []
            self.circle = []
        # 将矩形障碍区进行膨胀（先后顺序不能改变）
        self.rectangles = [self.create_expanded_rect(x[0], x[1], x[2], x[3]) for x in self.cube]
        self.rectangles.append(self.rectangle_obstacles)  # 将中心禁区矩形障碍物加入矩形障碍物列表

        gc.collect()

    # 辅助函数：由于原代码矩形检测未膨胀，这里手动对外扩充矩形顶点
    def create_expanded_rect(self, x_center, y_center, width, height, if_expand=True):
        hw = width / 2.0
        hh = height / 2.0
        if if_expand:
            hw += self.SAFE_MARGIN
            hh += self.SAFE_MARGIN
            
        return [
            (x_center - hw, y_center - hh),
            (x_center + hw, y_center - hh),
            (x_center + hw, y_center + hh),
            (x_center - hw, y_center + hh)
        ]

# 路径规划类
class PathPlan:
    def __init__(self, plan_data: PlanData, car):
        self.Data = plan_data
        self.my_car = car
        self.ready_path = []    # 规划好的路径
        gc.collect()

    # 路径规划主函数
    def plan_path(self, x1, y1, ignore_center_rect=False):
        # 获取当前障碍物列表（切片）
        circles = self.Data.circle[:]
        rects = self.Data.rectangles[:]
        self._p = [0.0,0.0,0.0,0.0]
        self._q = [0.0,0.0,0.0,0.0]
        # 如果特定状态激活，将原有的中心区域矩形障碍物移除
        # 根据 PlanData 的初始化，中心矩形障碍物是最后追加进去的
        if ignore_center_rect and len(rects) > 0:
            rects.pop(-1) 

        start = (float(self.my_car.x_current), float(self.my_car.y_current))
        end = (float(x1), float(y1))
        
        # 物理障碍物加安全裕量的总膨胀半径
        block_r = float(self.Data.OBSTACLE_R) + float(self.Data.SAFE_MARGIN)

        # 寻找距离不合法点最近的合法点逻辑函数
        def get_nearest_valid(p):
            # 将点限制在场地内
            px = max(0.0, min(p[0], float(self.Data.FIELD_W)))
            py = max(0.0, min(p[1], float(self.Data.FIELD_H)))
            valid_p = (px, py)
            
            if self._point_valid(valid_p, circles, rects, block_r):
                return valid_p
                
            # 若仍不合法（在障碍物内），从该点向外进行圆周辐射搜索
            search_radius = 2.0
            max_r = max(self.Data.FIELD_W, self.Data.FIELD_H)
            while search_radius < max_r:
                num_points = int(search_radius) + 8
                angle_step = 2.0 * _PI / num_points
                for i in range(num_points):
                    angle = i * angle_step
                    test_p = (px + math.cos(angle) * search_radius, py + math.sin(angle) * search_radius)
                    if self._point_valid(test_p, circles, rects, block_r):
                        return test_p
                search_radius += 2.0
            return valid_p
            
        # 更新起点和终点为合法的最近点
        start = get_nearest_valid(start)
        end = get_nearest_valid(end)

        # 检查直连
        if self._line_valid(start, end, circles, rects, block_r):
            self.ready_path = [[float(end[0]), float(end[1])]]

        # 初始化节点列表
        nodes = [start, end]
        # 添加圆形中继点 (核心修改部分)
        self._add_circle_nodes_fixed(nodes, circles, block_r)
        # 添加矩形中继点
        self._add_rectangle_nodes(nodes, rects, self.Data.SAFE_MARGIN)
        # 剔除无效点和重复点            
        nodes = self._unique_valid_nodes(nodes, circles, rects, block_r)

        n = len(nodes)
        dist = [self.Data.INF] * n
        prev = [-1] * n
        used = [False] * n
        dist[0] = 0.0

        # Dijkstra 算法实现
        for _ in range(n):
            u = -1
            best = self.Data.INF
            for i in range(n):
                if (not used[i]) and dist[i] < best:
                    best = dist[i]
                    u = i
            if u < 0 or u == 1:
                break
            used[u] = True

            for v in range(n):
                if used[v] or v == u:
                    continue
                if self._line_valid(nodes[u], nodes[v], circles, rects, block_r):
                    w = self._distance(nodes[u], nodes[v])
                    if dist[u] + w < dist[v]:
                        dist[v] = dist[u] + w
                        prev[v] = u

        if prev[1] < 0:
            return []

        # 重建路径
        path = []
        i = 1
        while i >= 0:
            path.append(nodes[i])
            i = prev[i]
        path.reverse()
        
        self.ready_path = self._path_to_list(self._smooth_path(path, circles, rects, block_r))
        # 删除起点
        self.ready_path.pop(0)

    # 初始化圆形障碍物列表
    def _normalize_points(self, points):
        out = []
        if not points: return out
        for p in points:
            if len(p) >= 2:
                out.append((float(p[0]), float(p[1])))
        return out

    # 初始化矩形障碍物列表
    def _normalize_rectangles(self,rects):
        if not rects: return []
        if len(rects) == 4 and len(rects[0]) == 2 and isinstance(rects[0][0], (int, float)):
            return [self._sort_polygon(self._normalize_points(rects))]
        out = []
        for rect in rects:
            pts = self._normalize_points(rect)
            if len(pts) >= 4:
                out.append(self._sort_polygon(pts))
        return out

    # 对多边形顶点进行排序，确保顺时针或逆时针顺序
    def _sort_polygon(self, poly):
        cx, cy = 0.0, 0.0
        for p in poly: cx += p[0]; cy += p[1]
        cx /= len(poly); cy /= len(poly)
        items = []
        for p in poly: items.append((math.atan2(p[1] - cy, p[0] - cx), p))
        items.sort()
        out = []
        for item in items: out.append(item[1])
        return out

    # 围绕圆心生成 8 个中继点
    def _add_circle_nodes_fixed(self, nodes, circles, block_r):
        num_points = 8
        angle_step = 2.0 * _PI / num_points
        
        # 距离计算必须使得弦的距离中心距离大于 block_r 才能被视为有效线段
        # d * cos(angle_step / 2) > block_r  ==>  d = block_r / cos(angle_step / 2) + margin
        node_radius = block_r / math.cos(angle_step / 2.0) + 1.0
        
        for c in circles:
            for i in range(num_points):
                angle = i * angle_step
                # 根据角度计算中继点坐标
                px = c[0] + math.cos(angle) * node_radius
                py = c[1] + math.sin(angle) * node_radius
                nodes.append((px, py))

    # 将矩形的四个角点添加为中继点
    def _add_rectangle_nodes(self, nodes, rects, margin):
        """原代码中的矩形节点生成逻辑"""
        # 注意：这里的 rects 已经是扩展了 SAFE_MARGIN 后的外围多边形
        # 向外留 2.0 的安全扩展距离生成中继点，防止浮点数碰线
        d = 2.0
        for rect in rects:
            cx, cy = 0.0, 0.0
            for p in rect: cx += p[0]; cy += p[1]
            cx /= len(rect); cy /= len(rect)
            for p in rect:
                vx, vy = p[0] - cx, p[1] - cy
                l = math.sqrt(vx * vx + vy * vy)
                
                if l == 0.0: nodes.append(p)
                else: nodes.append((p[0] + vx / l * d, p[1] + vy / l * d))

    # 对所有生成的候选节点进行过滤和去重
    def _unique_valid_nodes(self, nodes, circles, rects, block_r):
        out = []
        for p in nodes:
            if not self._inside_field(p): continue
            if not self._point_valid(p, circles, rects, block_r): continue
            duplicate = True
            for q in out:
                if abs(p[0] - q[0]) < 0.001 and abs(p[1] - q[1]) < 0.001:
                    duplicate = False; break
            if duplicate: out.append(p)
        return out

    # 判断点p是否有效（不在任何障碍物内，并且在场地内）
    def _point_valid(self, p, circles, rects, block_r):
        if not self._inside_field(p): return False
        for c in circles:
            if self._distance(p, c) <= block_r: return False
        for rect in rects:
            if self._point_in_rect(p, rect): return False
        return True

    # 判断线段ab是否与任何障碍物相交（ab不穿过障碍物）
    def _line_valid(self, a, b, circles, rects, block_r):
        for c in circles:
            if self._dist_point_to_seg(c, a, b) <= block_r: return False
        for rect in rects:
            if self.line_cross_rect(a, b, rect): return False
        return True

    # 判断点p是否在场地内
    def _inside_field(self, p):
        return p[0] >= 0.0 and p[0] <= self.Data.FIELD_W and p[1] >= 0.0 and p[1] <= self.Data.FIELD_H

    # 计算两点间距离
    def _distance(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    # 计算点p到线段ab的距离
    def _dist_point_to_seg(self, p, a, b):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        den = dx * dx + dy * dy
        if den < 1e-6: return self._distance(p, a)
        t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / den
        if t < 0.0: t = 0.0
        elif t > 1.0: t = 1.0
        return self._distance(p, (ax + t * dx, ay + t * dy))

    # 向量ab与ac的叉积
    def _cross(self, a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    # 判断点p是否在多边形poly内
    def _point_in_rect(self,p,rect):
        eps = 0.001
        if p[0]>rect[0][0]-eps and p[0]<rect[2][0]+eps and p[1]<rect[2][1]+eps and p[1]>rect[0][1]-eps:
            return True
        return False

    # 判断线段ab是否与多边形poly相交
    # 判断线段ab是否与多边形poly相交
    def line_cross_rect(self, p1, p2, rect):
        x1, y1 = p1
        x2, y2 = p2
        minn ,_ ,maxx ,_ = rect
        x_min, y_min = minn
        x_max, y_max = maxx
        if max(x1, x2) < x_min or min(x1, x2) > x_max or max(y1,y2) < y_min or min(y1, y2) > y_max:
            return False
        dx = x2 - x1
        dy = y2 - y1
        # 关键优化：将 self 属性 赋值给 局部变量（只需查找一次 self）
        p = self._p
        q = self._q
        # 原地修改数值（复用内存，无分配）
        p[0] = -dx; p[1] = dx; p[2] = -dy; p[3] = dy
        q[0] = x1 - x_min; q[1] = x_max - x1
        q[2] = y1 - y_min; q[3] = y_max - y1
        u1 = 0.0
        u2 = 1.0
        for i in range(4):  # 这里的循环开销在 Micropython 中还能接受
            if p[i] == 0:
                if q[i] < 0:
                    return False
            else:
                t = q[i] / p[i]
                if p[i] < 0:
                    if t > u2:
                        return False
                    if t > u1:
                        u1 = t
                else:
                    if t < u1:
                        return False
                    if t < u2:
                        u2 = t
        return u1 <= u2

    # 判断路径是否需要平滑，如果需要则进行平滑处理
    def _smooth_path(self, path, circles, rects, block_r):
        if len(path) <= 2: return path
        out = [path[0]]; i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self._line_valid(path[i], path[j], circles, rects, block_r): break
                j -= 1
            out.append(path[j]); i = j
        return out

    # 将返回的坐标点转换为列表形式
    def _path_to_list(self, path):
        return [[p[0], p[1]] for p in path]
# 导航规划类
# 导航规划类
class NavigationPlan:
    def __init__(self, flash_sys, fan, plan_data: PlanData, car, state: StateMachine, order_manager, my_uart3, beep, art_protocol, angle_pid):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入路径规划数据对象
        self.plan_data = plan_data
        # 注入无刷负压控制对象
        self.my_fan = fan
        # 注入小车位置对象
        self.my_car = car
        # 注入状态机对象
        self.my_state = state
        # 注入无线通信对象
        self.my_uart3 = my_uart3
        # 注入指令管理对象
        self.my_order_manager = order_manager
        # 注入蜂鸣器对象
        self.my_beep = beep
        # 注入openart串口解析对象
        self.my_art_protocol = art_protocol
        # 注入角度环PID对象
        self.angle_pid = angle_pid

        # 速度规划相关常量
        self.min_start_v = self.flash_sys.find_value("min_start_v")  # type: int  # 最小制动速度
        self.long_v_max = self.flash_sys.find_value("long_v_max")    # type: int  # 长距离时的最大速度
        self.acc_coef = 0.0          # 加速距离系数
        self.acc_normal_coef = self.flash_sys.find_value("acc_normal_coef")     # 正常导航的加速距离系数
        self.dec_coef = self.flash_sys.find_value("dec_coef")          # 减速距离系数
        self.move_v_max = 0.0     # 根据物体种类选择搬运速度
        self.find_line_v_max = self.flash_sys.find_value("find_line_v_max")  # 光电管寻找边界时的最大速度
        self.move_v_max_T = self.flash_sys.find_value("move_v_max_T")# type: int  # 搬运网球时的最大速度
        self.move_v_max_S = self.flash_sys.find_value("move_v_max_S")# type: int  # 搬运沙包时的最大速度
        self.move_v_max_B = self.flash_sys.find_value("move_v_max_B")# type: int  # 搬运玩具熊时的最大速度  
        self.move_S_increment = self.flash_sys.find_value("move_S_increment")# type: int  # 搬运沙包时的速度增量
        self.waypoint_v = []  # type: list  # 目标速度列表

        # 路径规划相关变量
        self.target_x = 0.0         # type: float
        self.target_y = 0.0         # type: float
        self.target_v = 0.0           # type: float  # 目标速度
        self.v_peak = 0.0           # type: float  # 当前路径段的理论最高速度
        self.target_yaw = 0.0            # type: float
        self.turn_angle_target = 0.0     # type: float
        # 判断小车是否到达目标点的阈值
        self.final_threshold = self.flash_sys.find_value("final_threshold")  # type: float
        self.branch_threshold = self.flash_sys.find_value("branch_threshold")  # type: float
        self.finished_dist = 0.0    # type: float
        self.rest_dist = 0.0        # type: float
        self.usable_len = 0.0         # type: float  # 当前路径段的可用长度（扣除提前到达阈值后的剩余距离）
        self.segment_start_dist = 0.0   # 当前路径段的起始点与过渡点之间的距离
        self.d_acc = 0.0                # 当前路径段的加速距离
        self.d_dec = 0.0                # 当前路径段的减速距离
        # 用于搬运你物体时矫正里程计的误差
        self.error_x_T = self.flash_sys.find_value("error_x_T")       # type: float
        self.error_x_S = self.flash_sys.find_value("error_x_S")       # type: float
        self.error_x_B = self.flash_sys.find_value("error_x_B")       # type: float
        self.error_x = 0.0

        # 已到达的目标点索引
        self.aimed_point_index = 0    # type: int
        # 目标路径
        self.path = []      # type: list
        # 标志位
        self.arrive_flag = False            # type: bool  # 判断是否到达目标点标志位
        self.if_finish_turn = False         # type: bool  # 判断是否完成转角调整标志位
        self.if_send_path = False           # type: bool  # 判断是否向从车发送路径标志位
        self.if_finish_navigate = False     # type: bool  # 判断是否完成导航标志位
        self.if_near_line = False           # type: bool  # 判断是否接近边界标志位
        self.if_first_turn = True           # type: bool  # 判断是否先进行转角调整
        self.if_push_T = False              # type: bool  # 判断是否搬运网球标志位
        self.if_inside_sandbag = False      # type: bool  # 判断是否搬运沙包标志位
        self.move_state = _NAVIGATE
        
        self.fit_target_yaw = 0.0
        self.keep_x_or_y_v = True #True表示keepx速度，False表示keepx速度
        self.fitting_path_ = []#如果需要内收，存放内收后的路径
        self.fit_rest_dist = 0.0

    # 离线预计算速度表 (根据中继点附近曲率推算最佳过渡速度)
    def pre_calculate_profile(self, path: list):
        # 打开无刷负压风扇
        '''
        if self.current_object in ['R', 'P']:
            self.my_fan.set_fan_signal()
        else:
            self.my_fan.fan_off()
        '''
        self.path = path[:] # 复制路径列表
        self.path.insert(0, [self.my_car.x_current, self.my_car.y_current])  # 在路径前添加主车起点
        if len(self.path) < 2: return
        
        self.acc_coef = self.acc_normal_coef

        n = len(self.path)
        self.waypoint_v = [self.min_start_v] * n

        for i in range(1, n - 1):
            yaw_in = -math.atan2(-(self.path[i][0] - self.path[i-1][0]), self.path[i][1] - self.path[i-1][1]) * 180.0 / _PI
            yaw_out = -math.atan2(-(self.path[i+1][0] - self.path[i][0]), self.path[i+1][1] - self.path[i][1]) * 180.0 / _PI
            delta_yaw = abs(yaw_out - yaw_in)
            if delta_yaw > 180.0: delta_yaw = 360.0 - delta_yaw
            # 当航向角变化超过一定角度时，强制设定通过该点的最大速度
            speed_factor = max(0.0, 1.0 - (delta_yaw / 180.0))
            # 再缩放0.5系数，让速度更保守一些，增加过弯安全裕量
            self.waypoint_v[i] = self.min_start_v + speed_factor * (self.long_v_max - self.min_start_v) * 0.4

        # 【前向推演：固有加速距离限制】
        for i in range(0, n - 1):
            seg_dist = math.sqrt((self.path[i+1][0] - self.path[i][0])**2 + (self.path[i+1][1] - self.path[i][1])**2)
            # 同样扣除提前到达阈值，这部分距离不能用来加速
            threshold = self.final_threshold if (i == n - 2) else self.branch_threshold
            # 2.0为安全裕量
            usable_dist = max(0.0, seg_dist - threshold - 2.0)
            max_reachable_v = self.waypoint_v[i] + (usable_dist / self.acc_coef)
            if self.waypoint_v[i+1] > max_reachable_v:
                self.waypoint_v[i+1] = max_reachable_v

        # 【反向推演：固有刹车减速距离限制】
        for i in range(n - 2, -1, -1):
            seg_dist = math.sqrt((self.path[i+1][0] - self.path[i][0])**2 + (self.path[i+1][1] - self.path[i][1])**2)
            # 考虑最后一段和中间段不同的“提前到达”阈值作为刹车缓冲区的扣除
            threshold = self.final_threshold if (i == n - 2) else self.branch_threshold
            # 2.0为安全裕量
            safe_dist = max(0.0, seg_dist - threshold - 2.0)
            max_safe_v = self.waypoint_v[i+1] + (safe_dist / self.dec_coef)
            if self.waypoint_v[i] > max_safe_v:
                self.waypoint_v[i] = max_safe_v

        self.aimed_point_index = 0
        self.if_finish_navigate = False
        # 计算第一段路径的加减速参数
        self.plan_acc_dec()
        self.target_v = self.waypoint_v[0]
        # 初始目标角直接看向第一个点
        self.target_yaw = -math.atan2(-(self.path[1][0] - self.path[0][0]), self.path[1][1] - self.path[0][1]) * 180.0 / _PI
        # 固定系数（负压状态下）
        self.my_car.alpha_x = 0.926049
        self.my_car.alpha_y = 0.920833


    # 根据当前过渡距离计算加减速距离
    def plan_acc_dec(self):
        # 修正：应该测量到下一个目标点(aimed_point_index + 1)的实物距离作为这段的总长
        target_pt = self.path[self.aimed_point_index + 1]
        self.segment_start_dist = math.sqrt((target_pt[0] - self.my_car.x_current)**2 + (target_pt[1] - self.my_car.y_current)**2)

        v_start = self.waypoint_v[self.aimed_point_index]
        v_end = self.waypoint_v[self.aimed_point_index + 1]

        # 修正：判断当前段的死区阈值，将S型的终点提前，得到真正的加减速“可用空间”
        is_last_segment = (self.aimed_point_index == len(self.path) - 2)
        threshold = self.final_threshold if is_last_segment else self.branch_threshold
        self.usable_len = max(0.01, self.segment_start_dist - threshold - 2.0) # 2.0为安全裕量

        if self.usable_len <= 0.1:
            self.v_peak = v_end
            self.d_acc = 0.0
            self.d_dec = 0.0
            return

        v_cruise = float(self.long_v_max)

        # 基于绝对速度变化量反算理论需要的加减速物理距离
        d_acc_req = self.acc_coef * max(0.0, v_cruise - v_start)
        d_dec_req = self.dec_coef * max(0.0, v_cruise - v_end)
        
        # 空间是否充裕判定
        if d_acc_req + d_dec_req > self.usable_len:
            # 空间不足，触发削峰逻辑
            if self.acc_coef + self.dec_coef > 0:
                self.v_peak = (self.usable_len + self.acc_coef * v_start + self.dec_coef * v_end) / (self.acc_coef + self.dec_coef)
            else:
                self.v_peak = v_cruise
            self.v_peak = max(self.v_peak, max(v_start, v_end))
        else:
            # 空间极其充裕，直接锁死在最高速阶段巡航
            self.v_peak = v_cruise

        # 平滑加减速阶段衔接区域
        self.d_acc = self.acc_coef * max(0.0, self.v_peak - v_start)
        self.d_dec = self.dec_coef * max(0.0, self.v_peak - v_end)
        if self.d_acc + self.d_dec > self.usable_len and (self.d_acc + self.d_dec) > 0:
            scale = self.usable_len / (self.d_acc + self.d_dec)
            self.d_acc *= scale
            self.d_dec *= scale

    def _calculate_position_s_curve(self):
        # 多项式平滑插值函数 Gentle-Smoothstep
        def smoothstep(t):
            t = max(0.0, min(1.0, t))
            # k 是融合系数，范围 0 到 1
            # k = 0 就是原来的三次方程 (中间最陡)
            # k = 1 就是纯匀速直线 (没有加减速过渡)
            # 推荐使用 0.3 ~ 0.5 之间，这里默认用 0.4
            k = 0.5
            cubic = 3 * (t ** 2) - 2 * (t ** 3)
            return k * t + (1 - k) * cubic

        v_start = self.waypoint_v[self.aimed_point_index]
        v_end = self.waypoint_v[self.aimed_point_index + 1]

        if self.move_state == _MOVE:
            v_cruise = self.move_v_max
        else:
            v_cruise = self.long_v_max
        # 在搬运状态下，小车如果接近边界需要降低速度便于光电管寻线
        if self.move_state == _MOVE:
            
            near_line_threshold = 20.0  # 距离边界的阈值，单位：cm
            if self.my_car.y_current >= self.plan_data.FIELD_H - near_line_threshold and self.keep_x_or_y_v == False:
                ratio = (self.plan_data.FIELD_H - self.my_car.y_current) / near_line_threshold
            elif self.my_car.y_current <= near_line_threshold and self.keep_x_or_y_v == False:
                ratio = self.my_car.y_current / near_line_threshold
            elif self.my_car.x_current <= near_line_threshold and self.keep_x_or_y_v == True:
                ratio = self.my_car.x_current / near_line_threshold
            elif self.my_car.x_current >= self.plan_data.FIELD_W - near_line_threshold and self.keep_x_or_y_v == True:
                ratio = (self.plan_data.FIELD_W - self.my_car.x_current) / near_line_threshold
            else:
                ratio = 1.0
            # 使用平方映射，使得减速更加剧烈，在较远处就开始显著降速
            ratio = max(0.0, min(1.0, ratio))
            ratio = ratio * ratio * ratio
            v_target = self.find_line_v_max + (self.move_v_max - self.find_line_v_max) * ratio

            move_v_max = self.move_v_max
        
            if self.if_push_T and self.aimed_point_index >= 1:
                move_v_max *= 1.2
            elif self.if_inside_sandbag and self.aimed_point_index == 0 and len(self.path) > 2:
                move_v_max = self.move_v_max_S + self.move_S_increment
            v_target = self.find_line_v_max + (move_v_max - self.find_line_v_max) * ratio
            return v_target
        # s 直接基于我们之前算出的 usable_len 限制
        s = self.segment_start_dist - self.rest_dist
        s_usable = max(0.0, min(s, self.usable_len))  # 强制束缚在可用区间内

        if s_usable <= self.d_acc:
            if self.d_acc <= 1e-3: v_out = self.v_peak
            else: v_out = v_start + (self.v_peak - v_start) * smoothstep(s_usable / self.d_acc)
        elif s_usable >= self.usable_len - self.d_dec:
            if self.d_dec <= 1e-3: v_out = v_end
            else:
                s_dec = s_usable - (self.usable_len - self.d_dec)
                v_out = self.v_peak + (v_end - self.v_peak) * smoothstep(s_dec / self.d_dec)
        else:
            v_out = self.v_peak
            
        # 这里改为输出 float，有助于你的底层PID或者跟随器能取得更平滑的参考速度
        return max(float(self.min_start_v), min(float(v_cruise), float(v_out)))
        
    # 实时导航执行函数
    def navigate_step(self):
        """
        实时执行：包含闭环航向解算与速度规划
        """
        # 更新小车当前位置
        car_x = self.my_car.x_current
        car_y = self.my_car.y_current

        # 更新小车到起点的距离
        self.finished_dist = math.sqrt((self.path[0][0] - car_x)**2 + (self.path[0][1] - car_y)**2)
        
        if self.aimed_point_index >= len(self.path) - 1:
            self.target_v = 0
            return self.target_v, self.target_yaw
        target_pt = self.path[self.aimed_point_index + 1]
        self.rest_dist = math.sqrt((target_pt[0] - car_x)**2 + (target_pt[1] - car_y)**2)
        if self.move_state == _MOVE: 
            p0 = self.path[self.aimed_point_index]
            p1 = self.path[self.aimed_point_index + 1]
            self.target_yaw = -math.atan2(-(p1[0] - p0[0]), p1[1] - p0[1]) * 180.0 / _PI
            if self.fitting_path_:
                target_pt = self.fitting_path_[self.aimed_point_index + 1]
                self.fit_rest_dist = math.sqrt((target_pt[0] - car_x)**2 + (target_pt[1] - car_y)**2)
                self.fit_target_yaw = -math.atan2(-(target_pt[0] - car_x), target_pt[1] - car_y) * 180.0 / _PI
            else:
                if self.keep_x_or_y_v:self.fit_rest_dist = abs(target_pt[0] - car_x)
                else:self.fit_rest_dist = abs(target_pt[1] - car_y)
        else:
            # =======================================================
            # 2. 闭环航向角解算模块
            # =======================================================
            self.target_yaw = -math.atan2(-(target_pt[0] - car_x), target_pt[1] - car_y) * 180.0 / _PI
        
        # =======================================================
        # 速度控制模块
        # =======================================================
        self.target_v = self._calculate_position_s_curve()
        # 输出限幅在 [-180, 180] 内
        if self.target_yaw > 180: self.target_yaw -= 360
        elif self.target_yaw < -180: self.target_yaw += 360
        
        # =======================================================
        # 到达判断
        # =======================================================
        is_last_segment = (self.aimed_point_index == len(self.path) - 2)
        rest_dist=self.rest_dist

        diff = 0.0
        if not self.if_first_turn:
            # 在未完成转角调整时，持续进行转角调整
            diff = abs(self.turn_angle_target - self.my_car.now_yaw * 180 / _PI)
            if diff > 180.0:
                diff = 360.0 - diff

        # 此时转角已完成转换成小角度模式
        if diff <= 1.5:
            self.angle_pid.choose_high_angle_mode(False)

        if_finish_turn = (diff <= 1.5) or (self.if_first_turn)

        if self.move_state == _MOVE: 
            rest_dist=self.fit_rest_dist

        if not is_last_segment and rest_dist <= self.branch_threshold:
            self.aimed_point_index += 1
            # 计算当前路径的加减速参数
            self.plan_acc_dec() 
            # pid积分清零
            self.my_car.reset_pid_integral()
            if self.move_state != _MOVE:
                target_pt = self.path[self.aimed_point_index + 1]
                self.rest_dist = math.sqrt((target_pt[0] - car_x)**2 + (target_pt[1] - car_y)**2)
        elif is_last_segment and rest_dist <= self.final_threshold and if_finish_turn:
            # 清空上一次小车速度
            self.my_car.clear_last_car_speed()
            self.angle_pid.choose_high_angle_mode(False)
            # 重置导航标志位
            self.if_finish_navigate = True
            self.stop()

    # 停止小车运动
    def stop(self):
        self.target_v = 0
        self.target_yaw = 0

    # 按照传入路径及进行惯性导航
    # 如果传入的目标转角不为none，则进行转角规划，否则不进行转角规划（用于路径点之间的过渡）
    def navigate(self, path = None, target_turn_angle = None, if_high_angle = False, if_first_turn = True):
        # 先进行转角调整使得路径规划与导航更稳定
        if self.if_finish_navigate == False:
            if self.if_finish_turn == False:
                if target_turn_angle is not None:
                    self.if_high_angle = if_high_angle
                    self.if_first_turn = if_first_turn

                    if if_high_angle and not self.angle_pid.if_high_angle:
                        self.angle_pid.choose_high_angle_mode(True)

                    self.target_v = 0
                    self.turn_angle_target = target_turn_angle
                    # 通过角度环限幅削弱转角调整的力度，帮助小车稳定完成转角调整
                    self.my_car.angle_pid.pwmout_limitmax = self.my_car.angle_pid.low_pwmout_limitmax
                    
                    # 在未完成转角调整时，持续进行转角调整
                    diff = abs(self.turn_angle_target - self.my_car.now_yaw * 180 / _PI)
                    if diff > 180.0:
                        diff = 360.0 - diff

                    if diff <= 1.5 or if_first_turn == False:
                        if if_first_turn:
                            # 换回小角度的角度环模式，帮助小车稳定完成转角调整
                            self.angle_pid.choose_high_angle_mode(False)

                        # 若不传入路径则当前导航已完成
                        if path is None:
                            self.if_finish_navigate = True
                        else:
                            self.if_finish_turn = True
                            self.pre_calculate_profile(path)
                else:
                    self.turn_angle_target = self.my_car.now_yaw * 180.0 / _PI
                    if path is None:
                        # 处理传入路径和角度都为空的情况
                        self.if_finish_navigate = True
                    else:
                        # 如果没有目标转角，直接认为转角调整完成
                        self.if_finish_turn = True  
                        self.pre_calculate_profile(path)
                    # 没有进行转角调整也要恢复原状
                    self.my_car.angle_pid.pwmout_limitmax = self.my_car.angle_pid.high_pwmout_limitmax
                    return 
            else:
                self.navigate_step()
        else:
            self.stop()
            self.if_finish_turn = False
            self.aimed_point_index = 0
            self.path.clear()
                
    # 重置导航及速度规划相关标志位
    def reset_navigate(self):
        self.target_v = 0.0
        self.if_finish_turn = False
        self.if_finish_navigate = False
        self.finished_dist = 0.0
        self.aimed_point_index = 0
        self.dec_counter = 0
        self.dec_start_v = 0.0
        self.path.clear()
        self.angle_pid.choose_high_angle_mode(False)
        self.if_push_T = False
        self.if_inside_sandbag = False

    # 重置小车导航姿态角
    def reset_navigate_angle(self):
        self.turn_angle_target = self.my_car.now_yaw * 180.0 / _PI