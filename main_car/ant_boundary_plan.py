import math
import gc
import time
class BoundaryPathPlanner:
    def __init__(self, plan_data, car, my_plan,flash_sys):
        self.Data = plan_data
        self.my_car = car
        self.my_plan = my_plan
        self.flash_sys = flash_sys
        self.sigal_swell_size = self.flash_sys.find_value("sigal_swell_size")#单向膨胀
        self.bothway_swell_size = self.flash_sys.find_value("bothway_swell_size")#双向膨胀
        self.MOVE_SAFE_MARGIN = self.flash_sys.find_value("MOVE_SAFE_MARGIN")#四周膨胀半径
        self.SAFE_MARGIN = self.flash_sys.find_value("SAFE_MARGIN")#四周膨胀半径
        self.near_area = self.flash_sys.find_value("NEAR_AREA")
        self.avoid_width = self.flash_sys.find_value("AVOID_WIDTH")
        self.forward_push_value = self.flash_sys.find_value("FORWARD_PUSH_VALUE")
        self.rects = []
        self.rectangles = []
        self.special_push = False
        self.circle = []
        self.fixed_obj = []
        self.ready_path = []
        self.judge_start_ticks = 0
        self.first_run = True
        self.last_swell_direct = None
        self.last_skip_idx = None
        self.xx = 0
        self.yy = 0
        self.direction = 0
        self.swell_size = 0
        self._p = [0.0,0.0,0.0,0.0]
        self._q = [0.0,0.0,0.0,0.0]
        gc.collect()
    def swell_rect(self,rect,swell_angle):
        out = []
        cx, cy = 0.0, 0.0
        if swell_angle == 1 or swell_angle == -1:
            for p in rect:
                cx += p[0]
                cy += p[1]
            cx /= len(rect)
            cy /= len(rect)
            _, right = self._forward_right(self._normalize_dir(self.direction))
        for p in rect:
            x, y = float(p[0]), float(p[1])
            if swell_angle == -90:
                if x < rect[0][0] + 0.001:
                    x -= self.swell_size
            elif swell_angle == 0:
                if y > rect[0][1] + 0.001:
                    y += self.swell_size
            elif swell_angle == 90:
                if x > rect[0][0] + 0.001:
                    x += self.swell_size
            elif swell_angle == 180:
                if y < rect[2][1] - 0.001:
                    y -= self.swell_size
            elif swell_angle == 1:
                side = (x - cx) * right[0] + (y - cy) * right[1]
                if side > 0.001:
                    x += right[0] * self.swell_size
                    y += right[1] * self.swell_size
                elif side < -0.001:
                    x -= right[0] * self.swell_size
                    y -= right[1] * self.swell_size
            elif swell_angle == -1:
                side = (x - cx) * right[0] + (y - cy) * right[1]
                if side > 0.001:
                    x += right[0] * self.swell_size
                    y += right[1] * self.swell_size
                elif side < -0.001:
                    x -= right[0] * self.swell_size
                    y -= right[1] * self.swell_size
            out.append((x, y))
        if self.direction == 0 and out[2][1] <= self.yy:return []
        elif self.direction == 180 and out[0][1] >= self.yy:return []
        elif self.direction == 90 and out[2][0] <= self.xx:return []
        elif self.direction == -90 and out[0][0] >= self.xx:return []
        return out
    def special_swell_barriers(self, objects_, swell_angle,skip_idx=None, direction=None,generate_new_obj = True):
        if swell_angle == 1 or swell_angle== -1:self.swell_size = self.bothway_swell_size
        else:self.swell_size = self.sigal_swell_size
        circle_r = float(self.Data.OBSTACLE_R)
        safe_margin = self.MOVE_SAFE_MARGIN
        circles = self.Data.circle
        raw_rects = self.Data.cube
        objects = objects_ if objects_ else []
        rects = []
        self.direction = direction
        def make_rect(cx, cy, half_w, half_h):
            return [
                (cx - half_w, cy - half_h),
                (cx + half_w, cy - half_h),
                (cx + half_w, cy + half_h),
                (cx - half_w, cy + half_h)
            ]
        # True表示进入了新一轮物体集合，必须替换旧缓存，不能继续append。
        # 数量不一致时即使调用方传False也自动重建，避免使用错位的旧缓存。
        if generate_new_obj or len(self.fixed_obj) != len(objects):
            self.fixed_obj.clear()
            for obj_idx in range(len(objects)):
                obj = objects[obj_idx]
                if len(obj) >= 4:
                    cx, cy = float(obj[0]), float(obj[1])
                    half_w = float(obj[2]) / 2.0 + safe_margin
                    half_h = float(obj[3]) / 2.0 + safe_margin
                    self.fixed_obj.append(make_rect(cx, cy, half_w, half_h))
                else:
                    # 保持fixed_obj与objects索引一一对应，skip_idx才不会错位。
                    self.fixed_obj.append(None)
        # 第一次只缓存未定向膨胀的基础矩形
        if self.first_run:
            for circle in circles:
                if len(circle) >= 2:
                    cx, cy = float(circle[0]), float(circle[1])
                    self.circle.append(
                        make_rect(cx, cy,
                                circle_r + safe_margin,
                                circle_r + safe_margin)
                    )
            for rect in raw_rects:
                if len(rect) >= 4:
                    cx, cy = float(rect[0]), float(rect[1])
                    half_w = float(rect[2]) / 2.0 + safe_margin
                    half_h = float(rect[3]) / 2.0 + safe_margin
                    self.rectangles.append(
                        make_rect(cx, cy, half_w, half_h)
                    )
            self.first_run = False
        # 每轮重新定向膨胀和按车位置过滤
        for obj_idx in range(len(objects)):
            if skip_idx is not None and obj_idx == skip_idx:continue
            obj = self.fixed_obj[obj_idx]
            if obj is None:continue
            new = self.swell_rect(obj,swell_angle)
            if new:rects.append(new)
        for rect in self.circle:
            new = self.swell_rect(rect, swell_angle)
            if new:rects.append(new)
        for rect in self.rectangles:
            new = self.swell_rect(rect, swell_angle)
            if new:rects.append(new)
        self.rects = rects

    def plan_move(self, direction, swell_dir, objects,x=None,y=None,skip_idx=None,limit_angle = None,generate_new_obj = True):
        if x is None or y is None:self.xx,self.yy=self.my_car.x_current,self.my_car.y_current
        else: self.xx,self.yy=x,y
        self.special_swell_barriers(objects, swell_dir,skip_idx, direction,generate_new_obj)
        self.ready_path = self.plan_one_turn(direction,limit_angle)
        return self.ready_path
    def plan_one_turn(self, direction,limit_angle):
        path_left = self._plan_one_turn_with_avoid(direction, -1)
        path_right = self._plan_one_turn_with_avoid(direction, 1)
        def calculate_angle(path,angle):
            if not path:return [],0
            push_yaw = math.atan2(path[1][0] - path[0][0], path[1][1] - path[0][1]) * 180.0 / math.pi
            push_angle = abs(push_yaw - direction)
            if push_angle > 180:
                push_angle = 360 - push_angle
            push_angle = abs(push_angle)
            if push_angle>angle:return [],0
            return path,push_angle
        if limit_angle:
            path_left,angle_l = calculate_angle(path_left,limit_angle)
            path_right,angle_r = calculate_angle(path_right,limit_angle)
        else:
            path_left,angle_l = calculate_angle(path_left,90)
            path_right,angle_r = calculate_angle(path_right,90)
        if not path_left:return path_right
        if not path_right:return path_left
        if self._path_cost(path_left,angle_l) <= self._path_cost(path_right,angle_r):return path_left
        return path_right
    def _plan_one_turn_with_avoid(self, direction, avoid_dir):
        direction = self._normalize_dir(direction)
        avoid_dir = 1 if avoid_dir >= 0 else -1
        start = (float(self.xx), float(self.yy))
        rects = self.rects
        start = self._nearest_valid(start, rects)
        direct_end = self._project_to_boundary(start, direction)
        if self._move_allowed(start, direct_end, direction, avoid_dir) and self._line_valid(start, direct_end, rects):
            return self.my_plan._path_to_list([start, direct_end])
        nodes = []
        fwd, right = self._forward_right(direction)
        for rect in rects:
            p = self._avoid_node(rect, direction, avoid_dir)
            if not self._ahead_or_level(start, p, direction):
                continue
            if not self._same_avoid_side_or_level(start, p, direction, avoid_dir):
                continue
            side_dist = abs((p[0] - start[0]) * right[0] + (p[1] - start[1]) * right[1])
            self._insert_sorted_node(nodes, side_dist, p)
        start_side = start[0] * right[0] + start[1] * right[1]
        for i in range(len(nodes)):
            ref_node = nodes[i][1]
            ref_side = ref_node[0] * right[0] + ref_node[1] * right[1]
            for j in range(i + 1):
                aim = nodes[j][1]
                aim_side = aim[0] * right[0] + aim[1] * right[1]
                den = aim_side - start_side
                if abs(den) < 0.000001:
                    continue
                t = (ref_side - start_side) / den
                if t < 0.0:
                    continue
                p = (start[0] + (aim[0] - start[0]) * t,
                     start[1] + (aim[1] - start[1]) * t)
                if not self.my_plan._inside_field(p):
                    continue
                if self._one_turn_candidate_cost(start, p, direction, avoid_dir, rects) < self.Data.INF:
                    return self.my_plan._path_to_list([start, p, self._project_to_boundary(p, direction)])
        return []

    def _insert_sorted_node(self, nodes, side_dist, p):
        item = [side_dist, p]
        idx = 0
        while idx < len(nodes) and nodes[idx][0] <= side_dist:
            idx += 1
        nodes.insert(idx, item)
    def _avoid_node(self, rect, direction, avoid_dir):
        d = 1.0
        if direction == -90:
            if avoid_dir == 1:return (rect[2][0] + d, rect[2][1] + d)
            else:return (rect[1][0] + d, rect[1][1] - d)
        elif direction == 90:
            if avoid_dir == 1:return (rect[0][0] - d, rect[0][1] - d)
            else:return (rect[3][0] - d, rect[3][1] + d)
        elif direction == 0:
            if avoid_dir == 1:return (rect[1][0] + d, rect[1][1] - d)
            else:return (rect[0][0] - d, rect[0][1] - d)
        else:
            if avoid_dir == 1:return (rect[3][0] - d, rect[3][1] + d)
            else:return (rect[2][0] + d, rect[2][1] + d)

    def _one_turn_candidate_cost(self, start, p, direction, avoid_dir, rects):
        if not self._point_valid(p, rects):
            return self.Data.INF
        end = self._project_to_boundary(p, direction)
        if not self._move_allowed(start, p, direction, avoid_dir):
            return self.Data.INF
        if not self._move_allowed(p, end, direction, avoid_dir):
            return self.Data.INF
        if not self._line_valid(start, p, rects):
            return self.Data.INF
        if not self._line_valid(p, end, rects):
            return self.Data.INF
        return self.my_plan._distance(start, p) + self.my_plan._distance(p, end)

    def _path_cost(self, path,angle):
        if not path:return self.Data.INF
        cost = 0.0
        for i in range(len(path) - 1):cost += self.my_plan._distance(path[i], path[i + 1])
        cost += angle*angle
        return cost
    def _normalize_dir(self, direction):
        if direction in (0, 90, 180, -90):
            return int(direction)
        raise ValueError("direction must be one of 0, 90, 180, -90")

    def _forward_right(self, direction):
        if direction == 0:
            return (0.0, 1.0), (1.0, 0.0)
        if direction == 90:
            return (1.0, 0.0), (0.0, -1.0)
        if direction == 180:
            return (0.0, -1.0), (-1.0, 0.0)
        return (-1.0, 0.0), (0.0, 1.0)
    
    def _project_to_boundary(self, p, direction):#计算在爹刺痛方向上边界的投影点
        if direction == 0:
            return (p[0], self.Data.FIELD_H+20)
        if direction == 180:
            return (p[0], -20)
        if direction == 90:
            return (self.Data.FIELD_W+20, p[1])
        return (-20, p[1])

    def _nearest_valid(self, p, rects):
        print('P_M in rects')
        px = max(0.0, min(float(p[0]), self.Data.FIELD_W))
        py = max(0.0, min(float(p[1]), self.Data.FIELD_H))
        p = (px, py)
        if self._point_valid(p, rects):
            return p
        radius = 2.0
        max_r = max(self.Data.FIELD_W, self.Data.FIELD_H)
        while radius < max_r:
            count = int(radius) + 8
            for i in range(count):
                a = 2.0 * math.pi * i / count
                q = (px + math.cos(a) * radius, py + math.sin(a) * radius)
                if self._point_valid(q, rects):return q
            radius += 2.0
        return p

    def _point_valid(self, p, rects):
        if not self.my_plan._inside_field(p):
            return False
        for rect in rects:
            if self._point_in_rect(p, rect):
                return False
        return True
    def _point_in_rect(self,p,rect):
        eps = 0.001
        if p[0]>rect[0][0]-eps and p[0]<rect[2][0]+eps and p[1]<rect[2][1]+eps and p[1]>rect[0][1]-eps:
            return True
        return False
    def _move_allowed(self, a, b, direction, avoid_dir):
        fwd, right = self._forward_right(direction)
        dx, dy = b[0] - a[0], b[1] - a[1]
        forward_len = dx * fwd[0] + dy * fwd[1]
        side_len = dx * right[0] + dy * right[1]
        return forward_len >= -0.001 and side_len * avoid_dir >= -0.001

    def _ahead_or_level(self, start, p, direction):
        fwd, _ = self._forward_right(direction)
        dx, dy = p[0] - start[0], p[1] - start[1]
        return dx * fwd[0] + dy * fwd[1] >= -0.001
    def _same_avoid_side_or_level(self, start, p, direction, avoid_dir):
        _, right = self._forward_right(direction)
        dx, dy = p[0] - start[0], p[1] - start[1]
        return (dx * right[0] + dy * right[1]) * avoid_dir >= -0.001
    def _line_valid(self, a, b, rects):
        for rect in rects:
            if self.line_cross_rect(a, b, rect):
                return False
        return True
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
    
class objects_planner:
    def __init__(self,my_flash_sys,my_write, plan_data, car, my_plan, my_BoundaryPath : BoundaryPathPlanner):
        self.flash_sys = my_flash_sys
        self.my_write = my_write
        self.Data = plan_data
        self.my_car = car
        self.my_plan = my_plan
        self.my_BoundaryPath = my_BoundaryPath
        self.barrier = []
        self.now_objects = []
        self.target_score = []
        self.plan_target = []
        self.path = []
        self.target_objects = []
        self.best_path = [0,0]
        self.judge_state = 0#0:未开始，1:正在进行，2:已结束
        self.last_sandbag_idx = -1
        self.now_idx = 0
        self.judge_start_ticks = 0
        self.run_speed = self.flash_sys.find_value("long_v_max") * 0.656467 * 0.72 # 0.656467为速度转换系数，把long_v_max转换为理论的厘米每秒速度，0.72转换为实际
        self.nine_grid = [['','',''],
                          ['','',''],
                          ['','',''],]
        self.wideness={'T':4.0,'S':3.0,'E':3.0,'B':3.0,'W':3.0,}
        self.height={'T':4.0,'S':3.0,'E':3.0,'B':3.0,'W':3.0,}
        gc.collect()
    def set_barriers(self,barriers):
        for i in self.now_objects:
            w,h=self.wideness[i[0]],self.height[i[0]]
            barriers.append([i[1],i[2],w,h])
    def reset_judge(self):
        self.last_sandbag_idx = -1
        self.path = []
        self.target_objects = []
        self.now_objects = []
        self.judge_state = 0
        self.barrier = []
        self.best_path = [0,0]
        self.target_score = []
        self.plan_target = []
        self.special_push = False
        self.now_idx = 0

    def nine_grid_postion_to_idx(self, x, y=None):
        """Return [row, col] for an exact nine-grid center, or [] if absent."""
        if y is None:
            x, y = x
        center_x = self.Data.center_x
        center_y = self.Data.center_y
        length = self.Data.lenth
        if length <= 0:
            return []
        col = int(round((x - center_x) / length)) + 1
        row = int(round((y - center_y) / length)) + 1
        if row < 0 or row > 2 or col < 0 or col > 2:
            return []
        expected_x = center_x + (col - 1) * length
        expected_y = center_y + (row - 1) * length
        if abs(x - expected_x) > 1e-6 or abs(y - expected_y) > 1e-6:return []
        return [row, col]

    def nine_grid_idx_to_postion(self, idx, col=None):
        """Return [x, y] for a [row, col] index, or [] if the index is invalid."""
        if col is None:row, col = idx
        else:row = idx
        if row < 0 or row > 2 or col < 0 or col > 2:return []
        center_x = self.Data.center_x
        center_y = self.Data.center_y
        length = self.Data.lenth
        if length <= 0:return []
        return [center_x + (col - 1) * length,
                center_y + (row - 1) * length]

    def generate_nine_grid(self):
        """Fill the 3x3 grid with object kinds from snapped object coordinates."""
        self.nine_grid = [['', '', ''], ['', '', ''], ['', '', '']]
        for obj in self.now_objects:
            if not obj or len(obj) < 3:
                continue
            idx = self.nine_grid_postion_to_idx(obj[1],obj[2])
            if idx:
                self.nine_grid[idx[0]][idx[1]] = obj[0]
        return self.nine_grid

    def judge_side_in_nine_grid(self,obj,dir,k):
        """判断物体沿 dir 方向连续 k 步的九宫格是否为空（方法级定义，避免循环内反复创建函数对象）"""
        if not obj or len(obj) < 3:
            return False
        now_pt = self.nine_grid_postion_to_idx(obj[1],obj[2])
        if not now_pt:
            return False
        now_pt[0] += dir[0] * k
        now_pt[1] += dir[1] * k
        nine_grid = self.nine_grid
        while now_pt[0] < 3 and now_pt[0] >= 0 and now_pt[1] < 3 and now_pt[1] >= 0:
            if nine_grid[now_pt[0]][now_pt[1]] != '':
                return False
            now_pt[0] += dir[0] * k
            now_pt[1] += dir[1] * k
        return True

    def judge_side_in_nine_grid_idx(self,idx,dir,k):
        """判断九宫格索引 idx 沿 dir 方向连续 k 步是否为空"""
        if not idx or len(idx) < 2:
            return False
        idx = idx[:]
        idx[0] += dir[0] * k
        idx[1] += dir[1] * k
        nine_grid = self.nine_grid
        while idx[0] < 3 and idx[0] >= 0 and idx[1] < 3 and idx[1] >= 0:
            if nine_grid[idx[0]][idx[1]] != '':
                return False
            idx[0] += dir[0] * k
            idx[1] += dir[1] * k
        return True

    def find_nine_grid_blank(self,obj,push_dir,in_dir):
        """沿推动方向寻找九宫格空白区域，返回搬运矩形与步数"""
        now_pt = self.nine_grid_postion_to_idx(obj[1],obj[2])
        if not now_pt:
            return [None, 0]
        i = now_pt[:]
        i[0] -= push_dir[0]
        i[1] -= push_dir[1]
        num = 0
        k = push_dir[0] + push_dir[1]
        use_big_rect = True
        while i[0] < 3 and i[0] >= 0 and i[1] < 3 and i[1] >= 0:
            if self.nine_grid[i[0]][i[1]] != '':
                use_big_rect = False
                break
            num += k
            if self.judge_side_in_nine_grid_idx(i,in_dir,-1):
                # in_dir uses [row, col], while world coordinates use [x, y].
                target_edge = [self.Data.center_x-in_dir[1]*1.5*self.Data.lenth,
                               self.Data.center_y-in_dir[0]*1.5*self.Data.lenth]#反向寻找进入边界
                p2 = self.nine_grid_idx_to_postion(i)
                if in_dir[0] == 0:p2 = [target_edge[0],p2[1]]
                else:p2 = [p2[0],target_edge[1]]
                now_xy = self.nine_grid_idx_to_postion(now_pt)
                return [[[min(now_xy[0],p2[0]),min(now_xy[1],p2[1])],[max(now_xy[0],p2[0]),max(now_xy[1],p2[1])]],num]
            i[0] -= push_dir[0]
            i[1] -= push_dir[1]
        if use_big_rect:return [[],0]
        else :return [None,0]
    def if_special_push(self):
        num_ = 0
        for i in range(3):
            num_+=1
            line_has_S = False
            nine_grid = self.nine_grid
            for j in range(3):
                if nine_grid[i][j] == 'T':
                    return False,num_
                if not line_has_S:
                    if nine_grid[i][j] in ['S','E']:
                        line_has_S = True
                else:
                    if nine_grid[i][j] in ['B','W']:
                        return True,num_
            if line_has_S: return False,num_
        return False,num_
    def judge_object_character(self,objects,car_side):
        if self.judge_state == 0:
            self.judge_start_ticks = time.ticks_ms()
            t_state = time.ticks_ms()
            # Keep only records with the fields used by the planner.  A partial
            # detection must not make the state machine fail during indexing.
            self.now_objects = [obj for obj in (objects or [])
                                if isinstance(obj, (list, tuple)) and len(obj) >= 3]
            self.set_barriers(self.barrier)#将物体转化为障碍形式并存储在self.barrier中
            self.generate_nine_grid()
            # print("[judge][state0] cost {} ms".format(time.ticks_diff(time.ticks_ms(), t_state)))
            self.judge_state = 1
            return False
        elif self.judge_state == 1:#筛选出能直接搬运的物体
            #t_state = time.ticks_ms()
            idx=0
            self.target_objects = []
            if len(self.now_objects) == 2 and car_side == 'L':
                self.special_push,num = self.if_special_push()
                if self.special_push:#强制选择熊
                    lenth = self.Data.lenth
                    center_x = self.Data.center_x
                    center_y = self.Data.center_y
                    for target in self.now_objects:
                        if target[0] in ['B','W']:
                            cy = num * lenth + center_y - lenth*2
                            half_h = 6 + self.Data.SAFE_MARGIN
                            rect = [(center_x - lenth*1.6,cy - half_h),
                                    (target[1] - lenth*0.5,cy - half_h),
                                    (target[1] - lenth*0.5,cy + half_h),
                                    (center_x - lenth*1.6,cy + half_h),]
                            self.target_objects.append([idx,target[0],target[1],target[2],'L',rect,num])
                            self.judge_state = 2
                            return False
                        idx+=1
            idx = 0
            for target in self.now_objects:
                #t_target = time.ticks_ms()
                could_select = True
                if target[0] == 'S' or target[0] == 'E':
                    push_dir = [0,-1]#推动正方向
                    self.last_sandbag_idx += 1
                    target_side = 'R'
                    if car_side == 'R' or car_side == 'L':could_select = False
                elif target[0] == 'T':
                    push_dir = [1,0]#推动正方向
                    target_side = 'D'
                    if car_side == 'D' or car_side == 'U':could_select = False
                else:
                    push_dir = [0,1]#推动正方向
                    target_side = 'L'
                    if car_side == 'L' or car_side == 'R':could_select = False
                if car_side == 'L':in_dir = [0,1]#进入正方向
                elif car_side == 'R':in_dir = [0,-1]
                elif car_side == 'D':in_dir = [1,0]
                else: in_dir = [-1,0]
                if self.judge_side_in_nine_grid(target,in_dir,-1):
                    self.target_objects.append([idx,target[0],target[1],target[2],car_side,[],0])#序号，物体种类，x,y,目标边,空表示用原矩形
                if not self.judge_side_in_nine_grid(target,push_dir,1):could_select = False
                if could_select:
                    rect,num = self.find_nine_grid_blank(target,push_dir,in_dir)
                    if rect != None:
                        self.target_objects.append([idx,target[0],target[1],target[2],target_side,rect,num])#序号，物体种类，x,y,目标边
                idx+=1
                # print("[judge][state1] target {} cost {} ms".format(idx-1, time.ticks_diff(time.ticks_ms(), t_target)))
            # print("[judge][state1] total {} ms".format(time.ticks_diff(time.ticks_ms(), t_state)))
            self.judge_state = 2
            return False
        elif self.judge_state == 2:#计算每个目标物体的评分
            t_state = time.ticks_ms()
            side_to_dir = {'D':0,'L':90,'U':180,'R':-90}
            if self.now_idx>=len(self.target_objects): 
                self.judge_state = 3
            else:
                t_target = time.ticks_ms()
                i = self.target_objects[self.now_idx]
                score = 0
                dir,sdir=self.judge_push_direction(i[1])
                if dir < side_to_dir[car_side]+0.1 and dir > side_to_dir[car_side]-0.1:score+=self.my_BoundaryPath.forward_push_value
                # 根据物体的种类调整目标点的位置，S和E向前移动10，B和W向后移动10，T向上移动10
                sx,sy = i[2],i[3]
                if i[1] in ['S', 'E']:  
                    sx += 10.0
                elif i[1] in ['B', 'W']:
                    sx -= 10.0
                elif i[1] in ['T']:
                    sy -= 10.0 
                if car_side == i[4]:
                    #t_plan = time.ticks_ms()
                    if self.now_idx == 0:
                        path = self.my_BoundaryPath.plan_move(dir, sdir, self.barrier, sx, sy, skip_idx=i[0],generate_new_obj =True)
                    else:
                        path = self.my_BoundaryPath.plan_move(dir, sdir, self.barrier, sx, sy, skip_idx=i[0],generate_new_obj = False)
                    # print("[judge][state2] plan_move {} ms".format(time.ticks_diff(time.ticks_ms(), t_plan)))
                else:
                    has_planned = False
                    for j in range(self.now_idx):
                        if i[0] == self.target_objects[j][0]:
                            path = [[sx,sy]]+self.path[j]
                            has_planned = True
                    if not has_planned:
                        #t_plan = time.ticks_ms()
                        if self.now_idx == 0:
                            path = self.my_BoundaryPath.plan_move(dir, sdir, self.barrier, sx, sy, skip_idx=i[0],generate_new_obj =True)
                        else:
                            path = self.my_BoundaryPath.plan_move(dir, sdir, self.barrier, sx, sy, skip_idx=i[0],generate_new_obj = False)
                        # print("[judge][state2] plan_move {} ms".format(time.ticks_diff(time.ticks_ms(), t_plan)))
                push_distance,push_angle= 1000,90
                if (not path) or len(path) <= 1: 
                    self.path.append([])
                    score+=10000
                else:
                    if len(path) == 2:
                        p_,p__=path[0],path[1]
                        push_distance = self.calculate_distance(p_,p__)
                    else:
                        p_,p__,p___=path[0],path[1],path[2]
                        push_distance = self.calculate_distance(p_,p__)+self.calculate_distance(p__,p___)
                    push_yaw = math.atan2(p__[0] - p_[0], p__[1] - p_[1]) * 180.0 / math.pi
                    self.path.append(path[1:])
                    push_angle = abs(push_yaw - dir)
                    if push_angle > 180:
                        push_angle = 360 - push_angle
                #旋转加分
                if (i[1] == 'S' or i[1] == 'E'):
                    if i[4] !='R':score+=1000
                    if self.last_sandbag_idx == 0:score+=5000
                elif (i[1] == 'T') and i[4] !='D':score+=1000
                elif (i[1] == 'W' or i[1] == 'B') and i[4] !='L':score+=1000
                # 大角度搬运路径加分
                if abs(push_angle) > 55: 
                    score+=3000
                if car_side == i[4]:
                    dx_car = i[2] - self.my_car.x_current
                    dy_car = i[3] - self.my_car.y_current
                    distance_from_car = math.sqrt(dx_car * dx_car + dy_car * dy_car)

                else:
                    RECT = i[5]
                    if not RECT:#使用大矩阵
                        RECT = [self.Data.center_rect[0],self.Data.center_rect[3]]
                    # 只计算目标边对应的入口点（避免每物体创建字典与元组）
                    side = i[4]
                    if side == 'D':
                        px = (RECT[0][0]+RECT[1][0])/2; py = RECT[0][1]
                    elif side == 'L':
                        px = RECT[0][0]; py = (RECT[0][1]+RECT[1][1])/2
                    elif side == 'U':
                        px = (RECT[0][0]+RECT[1][0])/2; py = RECT[1][1]
                    else:
                        px = RECT[1][0]; py = (RECT[0][1]+RECT[1][1])/2
                    x1 = i[2] - px
                    x2 = self.my_car.x_current - px
                    y1 = i[3] - py
                    y2 = self.my_car.y_current - py
                    distance_from_car = math.sqrt(x1 * x1 + y1 * y1) + math.sqrt(x2 * x2 + y2 * y2)
                dis_score = 1500 / self.run_speed # 1500为行进途中每秒对应的分数
                score += push_distance + push_angle ** 2 / 5 +distance_from_car*dis_score
                self.my_write.write_str("object {} push_dis:{} angle:{} dis:{}\n".format(i[1], push_distance, push_angle*push_angle, distance_from_car*dis_score))
                self.target_score.append(score)
                self.now_idx+=1
                # print("[judge][state2] object {} score cost {} ms".format(i[1], time.ticks_diff(time.ticks_ms(), t_target)))
            # print("[judge][state2] total {} ms".format(time.ticks_diff(time.ticks_ms(), t_state)))
            return False
        elif self.judge_state == 3:#选择评分最低的物体作为目标
            new_path = []
            for i in range(len(self.target_score)):
                if self.target_score[i] == min(self.target_score):
                    self.plan_target = self.target_objects[i]
                    if self.path[i]:
                        raw_x, raw_y = self.plan_target[2], self.plan_target[3]
                        if self.plan_target[1] in ['S', 'E']:
                            raw_x += 10.0
                        elif self.plan_target[1] in ['B', 'W']:
                            raw_x -= 10.0
                        elif self.plan_target[1] == 'T':
                            raw_y -= 10.0
                        dx = self.path[i][0][0] - raw_x
                        dy = self.path[i][0][1] - raw_y
                        self.best_path = [dx,dy]
                if self.path[i]:
                    new_path.append(self.path[i][0])
            self.path = new_path
            # print("[judge][state3] total {} ms".format(time.ticks_diff(time.ticks_ms(), t_state)))
            # print("[judge][whole] flow total {} ms".format(time.ticks_diff(time.ticks_ms(), self.judge_start_ticks)))
            return True
    def calculate_distance(self,p1,p2):
        return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
    def judge_push_direction(self,sp):
        if sp=='T': return 0,-1
        elif sp=='S' or sp=='E': return -90,1
        elif sp=='B' or sp=='W': return 90,1
        else :return {}
