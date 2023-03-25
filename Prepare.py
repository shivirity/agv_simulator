import random
import numpy as np
import pandas as pd
import logging
import colorlog

from ast import literal_eval
from gurobipy import *


# set logger
log_colors_config = {
    'DEBUG': 'cyan',
    'INFO': 'green',
    'WARNING': 'yellow',
    'ERROR': 'red',
    'CRITICAL': 'bold_red',
}
logger = logging.getLogger("logger_pre")
logger.setLevel(logging.DEBUG)
sh = logging.StreamHandler()
sh.setLevel(logging.INFO)
stream_fmt = colorlog.ColoredFormatter(
    fmt="%(log_color)s[%(asctime)s] - %(filename)-8s - %(levelname)-7s - line %(lineno)s - %(message)s",
    log_colors=log_colors_config)
sh.setFormatter(stream_fmt)
logger.addHandler(sh)
sh.close()

random.seed(42)


def sp_uniform(t, size):
    """生成在线任务到达时间，从0开始"""
    r = list(np.cumsum([t for _ in range(size)]))
    r.pop(-1)
    r.insert(0, 0)
    return r


# 根据节拍生成任务到达时间列表
def scheduling(scheduling_property, cycle_time, size):
    online_list_352 = eval('sp_{}(cycle_time, size)'.format(scheduling_property))
    online_list_356 = eval('sp_{}(cycle_time, size)'.format(scheduling_property))
    return online_list_352, online_list_356


# 标记叉车初始位置
df_loc = pd.read_csv("config/Car.csv")
init_loc = df_loc['park'].tolist()
# init_loc = [364, 362, 360, 354, 66, 57, 135, 140]  # 叉车初始坐标


# 有向图成环搜索函数
def dfs(graph, n_i, color, circle_list, have_circle, circle):
    r = len(graph)
    color[n_i] = -1
    for j in range(r):	 # 遍历当前节点i的所有邻居节点
        if graph[n_i][j] != 0:
            if color[j] == -1:
                have_circle += 1
                circle.append(circle_list)
            elif color[j] == 0:
                circle_list.append(j)
                have_circle = dfs(graph, j, color, circle_list, have_circle, circle)
    color[n_i] = 1
    return have_circle


def findcircle(graph) -> list:
    """"""
    # color = 0 该节点暂未访问
    # color = -1 该节点访问了一次
    # color = 1 该节点的所有孩子节点都已访问,就不会再对它做DFS了
    r = len(graph)
    color = [0] * r
    have_circle = 0
    circle = []
    for i in range(r):	 # 遍历所有的节点
        circle_list = []
        if color[i] == 0:
            circle_list.append(i)
            have_circle = dfs(graph, i, color, circle_list, have_circle, circle)
    return circle


class AGV:
    def __init__(self, num, simul_loc, park_grid, task_list):
        self.ID = num
        self.status = 'idle'  # idle, busy, down, block具体状态代表的含义还需要统一
        self.task = None  # 实时任务编号task_list[0]
        self.tasklist = task_list  # 已规划任务清单
        self.route = []  # 从 任务库 中导出对应任务编号的任务路径序列，并将其赋予叉车route_list[self.task]
        self.location = simul_loc  # 位置节点编号self.route[2]
        self.last_loc = None  # 序列中上一目标位置编号self.route[0]
        self.next_loc = None  # 序列中下一目标位置编号self.route[2]
        self.park_loc = park_grid
        self.task_status = None  # 'get', 'put', 'return', None
        self.is_load = 0  # 0表示车上空载，1表示车上有货
        self.end_state = (0, False)  # 车辆任务结束状态，第一项表示结束时间，第二项表示车辆是否已完成所有任务
        self.waiting_time = 0  # 表示车辆因为控制策略而等待的时间

    def get_location(self):  # 考虑可以放在Problem类，依据AGV编号来获取实时位置
        return self.location

    def get_status(self):  # 获取AGV的实时状态
        return self.status


class Task:
    def __init__(self, num, arrival, task_type, start, end, state, car):
        self.num = num
        self.arrival = arrival  # 任务到达时间
        self.type = task_type  # streamline(online)记为 1 & warehouse(offline)记为 0
        self.start = start  # 任务取货位置
        self.end = end  # 任务卸货位置，None表示未分配库位
        self.state = state  # 任务状态，0表示未分配，1表示被分配但未执行，2表示正在执行，3表示已完成
        self.car = car  # 表示分配的叉车序号，None表示未被分配
        self.route_seq = None  # 储存任务对应的路径序列，从起点到取货点到卸货点为完整序列


class Problem:
    __speed = 1
    step = 2
    alpha = 0.001  # 一类误差产生的概率
    beta = 0.001  # 二类误差产生的概率
    cargo_time = 5  # 取、卸货的时间步长
    side_penalty = 5  # 边缘冲突惩罚时间步长

    def __init__(self, dict_map, dict_task, dict_agv, dict_task_order, scheduling_property, cycle_time, size):
        self.Map = dict_map
        self.Task = dict_task  # 储存任务信息，包括任务状态
        self.AGV = dict_agv  # 储存各AGV的实时位置信息
        self.Task_order = dict_task_order  # 储存车辆在线任务终点的候选节点列表
        # self.Route = []  # 储存正在执行的任务路径 似乎不太需要
        self.loc_error = 0  # 系统出现位置重合碰撞的次数
        self.side_error = 0  # 系统出现位置库位边缘碰撞的次数
        self.deadlock = {}  # 储存发生死锁的车辆闭环
        self.deadlock_times = 0  # 储存发生死锁的次数
        # self.deadlock_happen = False  # 每一步储存是否有死锁发生
        # self.deadlock_num = []  # 用于储存每一步发生的死锁编号
        self.move_status = [0, 0, 0, 0, 0, 0, 0, 0]  # 叉车下一步行进状态列表，0 代表前进，其余代表所需停留的步长，默认为全部前进
        # moving_success: 叉车实际的行进状态，成功前进则为True, 原地停留则为False单步更新，可能会因为误差导致与规划不一致
        self.moving_success = [True, True, True, True, True, True, True, True]
        self.instruction = [True, True, True, True, True, True, True, True]  # 用于储存控制策略函数的返回值
        self.online_task_arrival_352, self.online_task_arrival_356 = scheduling(scheduling_property, cycle_time, size)
        self.time = 0  # 系统全局时间步
        self.is_finished = False  # 判定仿真过程是否完成
        self.online_task_arrival = False  # 判断是否有任务到达
        self.next_locs = []  # 用于暂存车辆下一步的位置
        # self.not_assigned_task_buffer = []  # 用于储存待规划的任务编号，调用函数才会更新
        # self.not_performed_task_buffer = []  # 用于储存待执行的离线任务编号，调用函数才会更新
        self.deadlock_happen = False

        # route_controller
        self.controller = None

    @ property
    def loc_error_count(self):
        return self.loc_error

    @ property
    def side_error_count(self):
        avg_error_times = 6
        return round(self.side_error / avg_error_times)

    # 更新任务字典，加入在线任务
    def renew_task_online(self, dict_task_simultaneous):
        num = len(dict_task_simultaneous) - 8
        p = num
        if self.online_task_arrival_352:
            while self.time >= self.online_task_arrival_352[0]:
                num += 1
                dict_task_simultaneous[num] = Task(num, self.online_task_arrival_352[0], 1, 352, None, 0, None)
                self.online_task_arrival_352.pop(0)
                if not self.online_task_arrival_352:
                    break

        if self.online_task_arrival_356:
            while self.time >= self.online_task_arrival_356[0]:
                num += 1
                dict_task_simultaneous[num] = Task(num, self.online_task_arrival_356[0], 1, 356, None, 0, None)
                self.online_task_arrival_356.pop(0)
                if not self.online_task_arrival_356:
                    break

        if p == num:
            self.online_task_arrival = False
        else:
            self.online_task_arrival = True

    # 碰撞判断
    def collision_check(self):
        self.next_locs = None
        agvs_list = [self.AGV[i+1].next_loc if self.instruction[i] is True and self.move_status[i] == 0
                     else self.AGV[i+1].location
                     for i in range(len(self.AGV))]
        agvs_set = set(agvs_list)
        # location collision
        if len(agvs_set) < len(self.AGV):
            self.loc_error += len(self.AGV) - len(agvs_set)
            self.next_locs = list(agvs_list)
        # near collision
        for i in range(len(self.AGV)-1):
            for j in range(i+1, len(self.AGV)):
                if agvs_list[j] in self.Map[agvs_list[i]].conflict:
                    self.side_error += 1
                    self.next_locs = list(agvs_list)
        return

    # 碰撞更新
    def update_control_policy(self):
        """如果冲突，让冲突的车辆停一轮，状态改为stop"""
        next_locs_set = set(self.next_locs)
        loc_hash = {key: [] for key in next_locs_set}
        for loc_idx in range(len(self.next_locs)):
            loc_hash[self.next_locs[loc_idx]].append(loc_idx)
        col_pairs = [i for i in loc_hash.values() if len(i) > 1]
        for pair in col_pairs:
            assert isinstance(pair, list)
        col_agvs = set([i for j in col_pairs for i in j])
        for agv in col_agvs:
            if self.move_status[agv] == 0:
                self.move_status[agv] += 1

    # 死锁判断
    def deadlock_check(self):
        agvs_list_before = []
        agvs_list_after = []
        for i in range(len(self.AGV)):
            agvs_list_after.append(self.AGV[i+1].next_loc)
            agvs_list_before.append(self.AGV[i+1].location)
        nodes = []
        x = []
        y = []
        for i in range(len(self.AGV)):
            if agvs_list_before[i] not in nodes:
                nodes.append(agvs_list_before[i])
            x.append(nodes.index(agvs_list_before[i]))
            if agvs_list_after[i] not in nodes:
                nodes.append(agvs_list_after[i])
            y.append(nodes.index(agvs_list_after[i]))
        g = np.zeros((len(nodes), len(nodes)), dtype=int)
        for j in range(len(self.AGV)):
            g[x[j]][y[j]] = 1

        have_circle = findcircle(g)

        if have_circle:
            for circle in have_circle:
                if len(circle) > 1:
                    break
            else:
                # 只有一辆车进入死锁，即该车cur_loc == next_loc
                return

        if have_circle:
            self.deadlock_happen = True
            for i in range(len(have_circle)):
                if len(have_circle[i]) == 1:
                    continue
                self.deadlock_times += 1
                logger.error(f'deadlock occurs between {have_circle}')
                list_deadlock = [x.index(node) for node in have_circle[i]]
                self.deadlock[self.deadlock_times] = list_deadlock
                # self.deadlock_num.append(self.deadlock_times)
                # 强制车辆前进至下一位置，并停留n（两）步，模拟死锁处理时间
                for j in list_deadlock:
                    self.move_status[i] += 2
                    self.AGV[j+1].last_loc = self.AGV[j+1].location
                    self.AGV[j+1].location = self.AGV[j+1].next_loc
                    self.AGV[j+1].route.pop(0)
                    if len(self.AGV[j+1].route) >= 3:
                        self.AGV[j+1].next_loc = self.AGV[j+1].route[2]
                    else:
                        self.AGV[j+1].next_loc = self.AGV[j+1].location

    '''
    # 仿真器死锁解决策略，强制改变控制路径的下发使得系统达到一个人工搬运后得以畅通的状态
    def deadlock_solving(self):
        if self.deadlock_happen:
            for i in self.deadlock_num:
                for j in self.deadlock[i]:
                    self.instruction[j] = True
            self.deadlock_happen = False
            self.deadlock_num = []
            '''

    # 误差生成
    def error_generating(self):
        # Type 1: low battery
        for i in range(len(self.AGV)):
            if random.random() < self.alpha and self.move_status[i] == 0 and self.AGV[i+1].status == 'busy':
                logger.warning(f'[LOW BATTERY]: agv{i+1} at time {self.time}.')
                self.move_status[i] += 1
                self.AGV[i+1].status = 'stop_low_battery'

        # Type 2: blocked or broken
        for i in range(len(self.AGV)):
            if random.random() < self.beta and self.AGV[i+1].status == 'busy':
                logger.warning(f'[BLOCKED or BROKEN]: agv{i+1} at time {self.time}.')
                self.move_status[i] += 10
                self.AGV[i+1].status = 'down'

    # 叉车前进
    def agvs_move(self, instruction):
        """控制车辆移动"""
        for i in range(len(self.AGV)):
            if instruction[i] and self.move_status[i] == 0:   # 如果指令为True，即为前进一步
                self.moving_success[i] = True
                self.AGV[i+1].last_loc = self.AGV[i+1].location
                self.AGV[i+1].location = self.AGV[i+1].next_loc
                self.AGV[i+1].route.pop(0)
                if len(self.AGV[i+1].route) >= 3:
                    self.AGV[i+1].next_loc = self.AGV[i+1].route[2]
                else:  # 进入停靠点
                    assert self.AGV[i+1].location == self.AGV[i+1].park_loc
                    assert len(self.AGV[i+1].route) == 2, f'{i+1, self.AGV[i+1].route}'
                    assert not self.AGV[i+1].tasklist, f'{i+1}'

            elif not instruction[i] and self.move_status[i] == 0:  # instruction[i] and self.move_status != 0:
                if self.AGV[i+1].route:  # route为空则说明还没接到任务，控制器能判断出来
                    if self.AGV[i+1].location != self.AGV[i+1].park_loc:
                        self.AGV[i+1].status = 'waiting'  # 该前进但需要等待
                        self.AGV[i+1].waiting_time += self.step
                self.moving_success[i] = False
            else:  # 该前进但因误差导致无法前进 or 本就无法前进
                self.moving_success[i] = False
                self.move_status[i] -= 1  # 如果状态为停留，那么随着时间步的增加，需要将停留的步数减一

    # 叉车状态变更
    def agvs_status_change(self):
        """控制车辆状态变更（走，停，转弯，...）"""
        for i in range(len(self.AGV)):
            # 取货结束
            if self.AGV[i+1].status == 'get_arriving' and self.move_status[i] == 0:
                assert self.AGV[i + 1].location == self.Task[self.AGV[i + 1].task].start
                self.AGV[i+1].task_status = 'put'
                # self.AGV[i+1].is_load = 1
                if self.Map[self.Task[self.AGV[i+1].task].start].state is not None:
                    self.Map[self.Task[self.AGV[i+1].task].start].state -= 1
            # 中途改变
            if self.move_status[i] == 0 and self.AGV[i+1].status not in ('busy', 'idle', 'put_arriving', 'turning'):
                if self.AGV[i+1].next_loc != self.AGV[i+1].location:
                    if self.AGV[i+1].task < 0:  # on return
                        self.AGV[i + 1].status = 'idle'
                    else:
                        self.AGV[i + 1].status = 'busy'
                else:
                    self.AGV[i+1].status = 'idle'
            # 卸货结束
            if self.AGV[i+1].status == 'put_arriving' and self.move_status[i] == 0:
                assert self.AGV[i+1].location == self.Task[self.AGV[i+1].task].end
                # self.AGV[i+1].is_load = 0
                self.Map[self.Task[self.AGV[i+1].task].end].state += 1
                self.Task[self.AGV[i+1].task].state = 3  # 代表当前任务完成
                self.AGV[i+1].tasklist.pop(0)
                self.AGV[i+1].task = self.AGV[i+1].tasklist[0]
                if len(self.AGV[i+1].tasklist) == 1:
                    self.AGV[i+1].status = 'idle'
                    self.AGV[i+1].task_status = 'return'
                else:
                    # 叉车从上一个任务的卸货点出发执行任务
                    self.Map[self.Task[self.AGV[i+1].task].start].reservation = True
                    self.Map[self.Task[self.AGV[i+1].task].end].reservation = True
                    self.Task[self.AGV[i+1].task].state = 2
                    # 在线任务开始执行后将终点序列更新
                    if i <= 3 and self.Task[self.AGV[i+1].task].end == self.Task_order[i+1][0]:
                        self.Task_order[i+1].pop(0)
                    self.AGV[i+1].task_status = 'get'
                    self.AGV[i+1].status = 'busy'

    # 返回所有AGV的实时任务执行状态
    def get_agv_status(self):
        status_list = []
        for i in range(len(self.AGV)):
            status_list.append(self.AGV[i+1].status)
        return status_list

    # 返回所有AGV的实时任务执行状态
    def get_agv_task_status(self):
        task_status_list = []
        for i in range(len(self.AGV)):
            task_status_list.append(self.AGV[i+1].task_status)
        return task_status_list

    # 返回所有AGV的实时位置
    def get_agv_location(self):
        location_list = []
        for i in range(len(self.AGV)):
            location_list.append(self.AGV[i+1].location)
        return location_list

    # 返回所有AGV的实时任务编号
    def get_agv_task(self):
        simultaneous_task_list = []
        for i in range(len(self.AGV)):
            simultaneous_task_list.append(self.AGV[i+1].task)
        return simultaneous_task_list

    # 返回所有AGV的实时任务列表
    def get_agv_tasklist(self):
        simultaneous_tasklist_list = []
        for i in range(len(self.AGV)):
            simultaneous_tasklist_list.append(self.AGV[i + 1].tasklist)
        return simultaneous_tasklist_list

    # 返回待规划的任务编号
    def get_task_not_assigned(self):
        not_assign_list = []
        for i in range(1, len(self.Task) - 7):
            if self.Task[i].state == 0:
                not_assign_list.append(i)
        return not_assign_list

    # 返回待执行的任务编号
    def get_task_not_perform(self):
        not_perform_list = []
        for i in range(1, len(self.Task) - 7):
            if self.Task[i].state == 0 or self.Task[i].state == 1:
                not_perform_list.append(i)
        return not_perform_list

    # 返回未完成的离线任务编号
    def get_undo_offline_task(self):
        return [i for i in self.Task.keys() if 0 < i < 101 and self.Task[i].state != 3]

    # 返回货位情况
    def get_cargo_state(self):
        # 节点的库存配置状态，None代表道路节点，0 代表无货，>= 1 代表有货

        cargo_occupied = []
        grid_reserved = []
        for i in range(1, len(self.Map)+1):
            # todo 返回被占用的库存节点编号，如果需返回货物数量，另写
            if self.Map[i].state >= 1:
                cargo_occupied.append(i)
            if self.Map[i].reservation:
                grid_reserved.append(i)
        return cargo_occupied, grid_reserved

    # 车辆到达并取卸货的判断
    def arriving_identify(self):
        for i in range(len(self.AGV)):
            if self.AGV[i+1].task:
                task_now = self.Task[self.AGV[i+1].task]
                # 开始取货
                if self.AGV[i+1].location == task_now.start and self.AGV[i+1].is_load == 0 and \
                        self.AGV[i+1].status != 'get_arriving':
                    self.AGV[i+1].status = 'get_arriving'
                    self.AGV[i+1].is_load = 1  # 用于到达后的起步判断了
                    self.Map[self.Task[self.AGV[i+1].task].start].reservation = False
                    self.move_status[i] += self.cargo_time  # 等待时间加上一个取、卸货的时间
                # 开始卸货
                elif self.AGV[i+1].location == task_now.end and self.AGV[i+1].is_load == 1 and \
                        self.AGV[i+1].status != 'put_arriving':
                    self.AGV[i+1].is_load = 0  # 用于到达后的起步判断了
                    self.AGV[i+1].status = 'put_arriving'
                    self.Map[self.Task[self.AGV[i+1].task].end].reservation = False
                    # 完成当前任务, 将当前任务从列表中pop出，但task属性还是需要判断是否在执行return任务
                    # self.AGV[i+1].tasklist.pop(0)
                    self.move_status[i] += self.cargo_time  # 等待时间加上一个取、卸货的时间
                # 车辆到达
                elif self.AGV[i+1].next_loc == self.AGV[i+1].park_loc and len(self.AGV[i+1].tasklist) == 1:
                    self.AGV[i + 1].tasklist = []
                    self.AGV[i + 1].task = None
                    self.AGV[i + 1].task_status = None
            else:
                # 车辆任务结束
                if self.time > 10584 and self.AGV[i + 1].end_state[1] is False:
                    self.AGV[i + 1].end_state = (self.time, True)
            if self.AGV[i+1].location == self.AGV[i+1].park_loc and len(self.AGV[i+1].tasklist) > 1:
                # 叉车从停靠点出发执行任务
                if self.AGV[i+1].task < 0:
                    assert self.AGV[i+1].tasklist[0] < 0
                    self.AGV[i+1].tasklist.pop(0)
                    self.AGV[i+1].task = self.AGV[i+1].tasklist[0]
                self.Task[self.AGV[i+1].task].state = 2
                self.Map[self.Task[self.AGV[i+1].task].start].reservation = True
                self.Map[self.Task[self.AGV[i+1].task].end].reservation = True
                # 在线任务开始执行后将终点序列更新
                if i <= 3 and self.Task_order[i+1]:
                    if self.Task[self.AGV[i+1].task].end == self.Task_order[i+1][0]:
                        self.Task_order[i+1].pop(0)
                    self.AGV[i+1].status = 'busy'
                    self.AGV[i+1].task_status = 'get'

    # 车辆转弯的判断
    def turning_identify(self):
        for i in range(len(self.AGV)):
            if self.AGV[i+1].last_loc is not None:
                l_x = self.Map[self.AGV[i+1].last_loc].x
                l_y = self.Map[self.AGV[i+1].last_loc].y
                n_x = self.Map[self.AGV[i+1].next_loc].x
                n_y = self.Map[self.AGV[i+1].next_loc].y
                if abs(l_x-n_x) > 0.6 and l_y != n_y and self.AGV[i+1].status != 'turning':
                    self.AGV[i+1].status = 'turning'
                    self.move_status[i] += 1
                elif abs(l_x-n_x) > 0.6 and l_y != n_y and self.AGV[i+1].status == 'turning':
                    if self.AGV[i+1].task < 0:
                        self.AGV[i + 1].status = 'idle'
                    else:
                        self.AGV[i+1].status = 'busy'

    # 将任务及路径分配给叉车时，更新叉车属性：
    def update_car(self, task_dict):
        for i in range(1, len(self.AGV)+1):
            '''
            # 判断此时是否在执行返回任务，若否，task和tasklist都空或都有
            if len(self.AGV[i].tasklist) == 1:  # 或self.AGV[i].task_status == ’return‘
                self.AGV[i].task_status = 'get'
                self.AGV[i].task = None
            '''
            self.AGV[i].tasklist = list(task_dict[i])
            if self.AGV[i].tasklist:  # 判断列表不为空
                if self.AGV[i].task is None:  # 包括初始化  # 或self.AGV[i].task_status == None
                    assert len(self.AGV[i].route) == 0 or len(self.AGV[i].route) == 2, f'agv{i}'
                    self.AGV[i].task = self.AGV[i].tasklist[0]
                    self.AGV[i].task_status = 'get'
                    self.Map[self.Task[self.AGV[i].task].start].reservation = True
                    self.Map[self.Task[self.AGV[i].task].end].reservation = True
                    for j in self.AGV[i].tasklist:
                        if j == self.AGV[i].task:
                            if len(self.AGV[i].route) == 2:
                                self.AGV[i].route += list(self.Task[j].route_seq[1:])
                            else:
                                self.AGV[i].route += list(self.Task[j].route_seq)
                        else:
                            tmp = list(self.Task[j].route_seq)
                            self.AGV[i].route += tmp[1:]
                else:
                    temp = list(self.AGV[i].tasklist)
                    cur_task = temp.pop(0)
                    if self.AGV[i].route.count(self.Task[cur_task].end) > 1:
                        if cur_task > 0:  # 非返回任务
                            end_idx = [idx for idx in range(len(self.AGV[i].route)) if self.AGV[i].route[idx] == self.Task[cur_task].end]
                            first_get_idx = self.AGV[i].route.index(self.Task[cur_task].start)
                            cur_end_idx = [i for i in end_idx if i > first_get_idx][0]
                        else:  # 返回任务在第一个停靠点
                            cur_end_idx = self.AGV[i].route.index(self.Task[cur_task].end)
                    else:
                        cur_end_idx = self.AGV[i].route.index(self.Task[cur_task].end)
                    self.AGV[i].route = self.AGV[i].route[:cur_end_idx+1]  # delete undo task
                    for j in temp:
                        tmp = list(self.Task[j].route_seq)
                        self.AGV[i].route += tmp[1:]
                if self.AGV[i].task > 0:
                    self.Task[self.AGV[i].task].state = 2
                    self.Task[self.AGV[i].task].car = i
                    if self.AGV[i].task_status == 'get' and self.AGV[i].location != self.Task[self.AGV[i].task].start:
                        self.Map[self.Task[self.AGV[i].task].start].reservation = True
                        self.Map[self.Task[self.AGV[i].task].end].reservation = True
                    for j in range(1, len(self.AGV[i].tasklist)):
                        if self.AGV[i].tasklist[j] > 0:
                            self.Task[self.AGV[i].tasklist[j]].state = 1
                            self.Task[self.AGV[i].tasklist[j]].car = i
            else:  # 未被分配到任务
                assert self.AGV[i].task is None

            if self.AGV[i].route:
                if self.AGV[i].route[0] != self.AGV[i].location:
                    # already updated
                    assert self.AGV[i].route[0] == self.AGV[i].last_loc
                    assert self.AGV[i].next_loc == self.AGV[i].location or \
                           self.AGV[i].next_loc == self.AGV[i].route[2]
                else:
                    # update last_loc
                    if self.AGV[i].last_loc is None:  # 只在初始化时last_loc为None
                        self.AGV[i].route.insert(0, self.AGV[i].location)
                        self.AGV[i].last_loc = self.AGV[i].route[0]
                    else:
                        self.AGV[i].route.insert(0, self.AGV[i].last_loc)
                    # update next_loc
                    if len(self.AGV[i].route) >= 3:
                        self.AGV[i].next_loc = self.AGV[i].route[2]
                    else:
                        # only happens in returning
                        assert self.AGV[i].task < 0
                        self.AGV[i].next_loc = self.AGV[i].location
        logger.info('route updated.')

    # 将路径赋予对应任务时，更新任务的路径序列：
    def update_task(self):
        # todo 有记录的需要的话再写
        pass

    # 更新系统时间步长
    def update_step(self):
        self.time += self.step

    def fix_instruction(self):
        """冲突检测+策略修正"""
        # part1
        self.collision_check()
        # part2
        if self.next_locs is not None:
            self.update_control_policy()

    def finish_identify(self):
        if self.time <= 5400:
            self.is_finished = False
        else:
            for task in [i for i in self.Task.keys() if i > 0]:
                if self.Task[task].state != 3:
                    self.is_finished = False
                    break
            else:
                for agv in self.AGV.keys():
                    if self.AGV[agv].location != self.AGV[agv].park_loc:
                        self.is_finished = False
                        break
                else:
                    self.is_finished = True

    # 主流程
    def run_step(self):
        """
        主流程函数
        """
        # 根据任务到达情况更新task列表
        self.deadlock_check()
        self.turning_identify()
        self.arriving_identify()

        # 如果发生死锁，在deadlock_check中已解决，需要更新路径
        if self.deadlock_happen:
            self.controller.update_routes(routes={i: self.AGV[i+1].route[2:] for i in range(8)})

        if not self.time:
            self.instruction = self.controller.get_instruction(
                status_list=[True if i < 0.1 else False for i in self.move_status],
                loc_list=[self.AGV[i].location for i in range(1, 9)],
                undo_offline_tasks=self.get_undo_offline_task()
            )
        else:
            self.instruction = self.controller.get_instruction(
                status_list=[True if i < 0.1 else False for i in self.move_status],
                loc_list=[self.AGV[i].location for i in range(1, 9)],
                step_list=self.moving_success,
                undo_offline_tasks=self.get_undo_offline_task()
            )

        for agv in range(8):
            if len(self.AGV[agv+1].route) >= 3:
                assert self.AGV[agv+1].route[2] == self.controller.residual_routes[agv][0], f'{agv}'

        # 下发控制路径 step_list 上一步到底有没有走成功，转弯不算走，移动

        self.error_generating()  # 误差生成

        # 冲突检测+策略修正
        self.fix_instruction()
        self.agvs_move(self.instruction)
        self.agvs_status_change()
        self.update_step()
        self.finish_identify()
        self.deadlock_happen = False  # reset deadlock

    # 路径规划及任务调度
    # 路径规划及任务调度
    def route_scheduling(self, time, agv_max_task_num=2, candidate_num=5):
        class Path:
            start_node = []  # 起始节点（路径节点）
            end_node = []  # 终止节点（路径节点）
            static_congestion = []  # 静态拥挤度
            path_time = []  # 路径耗费时间（考虑转弯数量）
            Paths = [[]]  # 候选路径节点集

        stock_path = pd.read_csv("config/stock_path.csv")
        stock_entrance_path = {}
        for i in range(len(stock_path)):
            stock_entrance_path[stock_path.loc[i][0], stock_path.loc[i][1]] = stock_path.loc[i][2].split(', ')
            stock_entrance_path[stock_path.loc[i][0], stock_path.loc[i][1]] = [int(x) for x in stock_entrance_path[stock_path.loc[i][0], stock_path.loc[i][1]]]
        path_candidate = pd.read_csv("config/{}_all_path_candidate.csv".format(candidate_num))
        path_candidate['Paths'] = path_candidate['Paths'].apply(literal_eval)
        path_candidate['Route time'] = path_candidate['Route time'].apply(literal_eval)
        path_candidate['Static congestion'] = path_candidate['Static congestion'].apply(literal_eval)
        for i in range(len(path_candidate['Paths'])):
            for j in range(len(path_candidate.loc[i]['Paths'])):
                path_candidate.loc[i]['Paths'][j] = [int(x) for x in path_candidate.loc[i]['Paths'][j]]
        for i in range(len(path_candidate['Route time'])):
            for j in range(len(path_candidate.loc[i]['Route time'])):
                path_candidate.loc[i]['Route time'][j] = int(path_candidate.loc[i]['Route time'][j])
                path_candidate.loc[i]['Static congestion'][j] = int(path_candidate.loc[i]['Static congestion'][j])
        '''建立模型'''
        task_not_assigned = self.get_task_not_assigned()
        task_not_perform = self.get_task_not_perform()
        # 获取任务列表
        task_f_end = {}
        flow_task = [i for i in (task_not_perform + task_not_assigned) if i > 100]
        stock_task = [i for i in (task_not_perform + task_not_assigned) if i <= 100]
        flow_task = list(set(flow_task))
        stock_task = list(set(stock_task))
        stock_task_i = {}
        stock_task_i[1] = [i for i in stock_task if 1 <= i < 26]
        stock_task_i[2] = [i for i in stock_task if 26 <= i < 51]
        stock_task_i[3] = [i for i in stock_task if 51 <= i < 76]
        stock_task_i[4] = [i for i in stock_task if 76 <= i < 101]
        for i in range(1, 5):
            stock_task_i[i].sort(reverse=True)
        flow_task.sort(reverse=False)
        current = self.get_agv_task()
        current_task = current.copy()
        current_not_virtual_task = {}
        for i in range(len(current_task)):
            current_not_virtual_task[i + 1] = [current_task[i]]
            if current_task[i] is None:
                current_task[i] = - i - 1
                current_not_virtual_task[i + 1] = []
            if current_task[i] < 0:
                current_not_virtual_task[i + 1] = []
        agv_task = {}
        factual_s_task_num = {}
        for i in range(1, 5):
            factual_s_task_num[i] = 0
        for i in range(len(current_task)):
            if current_task[i] is None or current_task[i] <= 0:
                agv_task[i + 1] = 0
            else:
                agv_task[i + 1] = 1
                if 1 <= current_task[i] < 26:
                    factual_s_task_num[1] += 1
                elif 26 <= current_task[i] < 51:
                    factual_s_task_num[2] += 1
                elif 51 <= current_task[i] < 76:
                    factual_s_task_num[3] += 1
                elif 76 <= current_task[i] < 101:
                    factual_s_task_num[4] += 1
        # 定义变量
        V_f = [1, 2, 3, 4]  # 流水线区域的叉车
        V_s = [5, 6, 7, 8]  # 中转库区域的叉车
        candidate_area = {}
        m = []
        for v in V_f:
            candidate_area[v] = (self.Task_order[v]).copy()
            if len(candidate_area[v]) == 0:
                m.extend([v])
        for n in m:
            V_f.remove(n)
        agv_max_task = {}
        update_task_f_num = 0
        for v in V_f:
            agv_max_task[v] = min(len(candidate_area[v]), (agv_max_task_num - agv_task[v]))
            update_task_f_num += agv_max_task[v]
        if len(flow_task) >= update_task_f_num:
            task_f = flow_task[:update_task_f_num]
        else:
            task_f = flow_task

        task_s = []
        task_s_i = {}
        for i in range(1, 5):
            if len(stock_task_i[i]) >= (agv_max_task_num - factual_s_task_num[i]):
                task_s_i[i] = stock_task_i[i][:(agv_max_task_num - factual_s_task_num[i])]
                task_s += stock_task_i[i][:(agv_max_task_num - factual_s_task_num[i])]
            else:
                task_s_i[i] = stock_task_i[i]
                task_s += stock_task_i[i]
        '''考虑是否删掉对应车辆'''
        m = []
        for i in range(1, 5):
            if len(task_s_i[i]) == 0:
                for v in V_s:
                    if (1 + 25 * (i - 1)) <= current_task[v - 1] < (1 + 25 * i) or current_task[v - 1] is None or current_task[v - 1] < 0:
                        m.extend([v])
        m = list(set(m))
        for n in m:
            V_s.remove(n)
        R = [i for i in range(1, 5) if len(task_s_i[i]) > 0]

        _I_f = [-v for v in V_f]
        _I_s = [-v for v in V_s]
        I_f = task_f.copy()  # 流水线区域更新的任务
        I_s = task_s.copy()  # 中转库区域更新的任务
        I_f_0 = task_f.copy()
        for v in V_f:
            I_f_0.extend([current_task[v - 1]])  # 流水线区域更新的任务（包含每个叉车正在执行的任务）
        I_s_0 = task_s.copy()   # 中转库区域更新的任务（包含每个叉车正在执行的任务）
        for v in V_s:
            I_s_0.extend([current_task[v - 1]])  # 流水线区域更新的任务（包含每个叉车正在执行的任务）
        I_f_vir = task_f + _I_f  # 流水线区域更新的任务（包含每个叉车的最后一个虚拟任务）
        I_s_vir = task_s + _I_s  # 中转库区域更新的任务（包含每个叉车的最后一个虚拟任务）

        pathf_i_v = {}
        paths_i_v = {}
        qf_i_j_v = {}
        qs_i_j_v = {}
        P_f = list(range(candidate_num))
        P_s = list(range(candidate_num))
        Q_f = list(range(candidate_num))
        Q_s = list(range(candidate_num))

        # 流水线任务i对应叉车v的路径集更新
        for v in V_f:
            for i in (I_f + [-v] + current_not_virtual_task[v]):
                m = self.Map[self.Task[i].start].entrance
                if i < 0 or (len(current_not_virtual_task[v]) > 0 and i == current_not_virtual_task[v][0]):
                    n = self.Map[self.Task[i].end].entrance
                else:
                    # 先暂定对应叉车v都去向当前可放置货物的第一个节点
                    n = self.Map[candidate_area[v][0]].entrance
                    task_f_end[i, v] = candidate_area[v][0]
                    self.Task[i].end = candidate_area[v][0]
                for j in range(len(path_candidate)):
                    if path_candidate.loc[j][0] == m and path_candidate.loc[j][1] == n:
                        x = Path()
                        x.start_node = path_candidate.loc[j][0]
                        x.end_node = path_candidate.loc[j][1]
                        x.Paths = path_candidate.loc[j][2]
                        x.path_time = path_candidate.loc[j][3]
                        x.static_congestion = path_candidate.loc[j][4]
                        pathf_i_v[i, v] = x
                        break

        # 中转库任务i对应叉车v的路径集更新
        for v in V_s:
            for i in (I_s + [-v] + current_not_virtual_task[v]):
                for j in range(len(path_candidate)):
                    if path_candidate.loc[j][0] == self.Map[self.Task[i].start].entrance and path_candidate.loc[j][
                            1] == self.Map[self.Task[i].end].entrance:
                        x = Path()
                        x.start_node = path_candidate.loc[j][0]
                        x.end_node = path_candidate.loc[j][1]
                        x.Paths = path_candidate.loc[j][2]
                        x.path_time = path_candidate.loc[j][3]
                        x.static_congestion = path_candidate.loc[j][4]
                        paths_i_v[i, v] = x
                        break

        # 流水线任务i接任务j对应叉车v的路径集更新
        for v in V_f:
            for i in (I_f + [current_task[v - 1]]):
                for j in (I_f + [-v]):
                    if (i == current_task[v - 1]) or (i < 0):
                        p = self.Map[self.Task[i].end].entrance
                    else:
                        p = self.Map[task_f_end[i, v]].entrance
                    q = self.Map[self.Task[j].start].entrance
                    for m in range(len(path_candidate)):
                        if path_candidate.loc[m][0] == p and path_candidate.loc[m][1] == q:
                            x = Path()
                            x.start_node = path_candidate.loc[m][0]
                            x.end_node = path_candidate.loc[m][1]
                            x.Paths = path_candidate.loc[m][2]
                            x.path_time = path_candidate.loc[m][3]
                            if 0 < j <= i:
                                x.path_time = [1000] * candidate_num
                            x.static_congestion = path_candidate.loc[m][4]
                            qf_i_j_v[i, j, v] = x
                            break

        # 中转库任务i接任务j对应叉车v的路径集更新
        for v in V_s:
            for i in (I_s + [current_task[v - 1]]):
                for j in (I_s + [-v]):
                    for m in range(len(path_candidate)):
                        if path_candidate.loc[m][0] == self.Map[self.Task[i].end].entrance and path_candidate.loc[m][
                            1] == \
                                self.Map[self.Task[j].start].entrance:
                            x = Path()
                            x.start_node = path_candidate.loc[m][0]
                            x.end_node = path_candidate.loc[m][1]
                            x.Paths = path_candidate.loc[m][2]
                            x.path_time = path_candidate.loc[m][3]
                            if j >= i > 0:
                                x.path_time = [1000] * candidate_num
                            x.static_congestion = path_candidate.loc[m][4]
                            qs_i_j_v[i, j, v] = x
                            break

        model = Model("Task Assignment and Path Planning")  # construct the model object
        xf = {}
        xs = {}
        yf = {}
        ys = {}
        T = {}
        z = {}
        area = {}
        # Initialize variables
        for v in V_f:
            for i in (I_f + [-v] + current_not_virtual_task[v]):
                for p in P_f:
                    name1 = 'xf_' + str(i) + '_' + str(v) + '_' + str(p)
                    xf[i, v, p] = model.addVar(vtype=GRB.BINARY, name=name1)
        for v in V_s:
            for i in (I_s + [-v] + current_not_virtual_task[v]):
                for p in P_s:
                    name2 = 'xs_' + str(i) + '_' + str(v) + '_' + str(p)
                    xs[i, v, p] = model.addVar(vtype=GRB.BINARY, name=name2)
        for v in V_f:
            for i in (I_f + current_task):
                for j in (I_f + _I_f):
                    for q in Q_f:
                        name3 = 'yf_' + str(i) + '_' + str(j) + '_' + str(v) + '_' + str(q)
                        yf[i, j, v, q] = model.addVar(vtype=GRB.BINARY, name=name3)
        for v in V_s:
            for i in (I_s + current_task):
                for j in (I_s + _I_s):
                    for q in Q_s:
                        name4 = 'ys_' + str(i) + '_' + str(j) + '_' + str(v) + '_' + str(q)
                        ys[i, j, v, q] = model.addVar(vtype=GRB.BINARY, name=name4)
        for v in (V_f + V_s):
            name5 = 'T_' + str(v)
            T[v] = model.addVar(vtype=GRB.CONTINUOUS, name=name5)
        for r in R:
            for v in V_s:
                name6 = 'z_' + str(r) + '_' + str(v)
                z[r, v] = model.addVar(vtype=GRB.BINARY, name=name6)
        '''约束条件为在考虑路径拥堵度的情况下最小化流水线及中转库的最大完工时间'''
        solution1 = model.addVar(vtype=GRB.CONTINUOUS, name='solution1')
        solution2 = model.addVar(vtype=GRB.CONTINUOUS, name='solution2')
        for r in R:
            for v in V_s:
                name7 = 'area_' + str(r) + '_' + str(v)
                area[r, v] = model.addVar(vtype=GRB.INTEGER, name=name7)
        # 添加目标函数约束
        model.addConstrs((sum(
            xf[i, v, p] * pathf_i_v[i, v].path_time[p] * pathf_i_v[i, v].static_congestion[p] for i in (I_f + [-v] + current_not_virtual_task[v]) for
            p in P_f) + sum(
            yf[i, j, v, q] * qf_i_j_v[i, j, v].path_time[q] * qf_i_j_v[i, j, v].static_congestion[q] for i in
            (I_f + [current_task[v - 1]]) for j in (I_f + [-v]) for q in Q_f)) == T[v] for v in V_f)
        model.addConstrs((sum(
            xs[i, v, p] * paths_i_v[i, v].path_time[p] * paths_i_v[i, v].static_congestion[p] for i in (I_s + [-v] + current_not_virtual_task[v]) for
            p in P_s) + sum(
            ys[i, j, v, q] * qs_i_j_v[i, j, v].path_time[q] * qs_i_j_v[i, j, v].static_congestion[q] for i in
            (I_s + [current_task[v - 1]]) for j in (I_s + [-v]) for q in Q_s)) == T[v] for v in V_s)
        model.addConstrs(T[v] <= solution1 for v in V_f)
        model.addConstrs(T[v] <= solution2 for v in V_s)
        # 添加目标函数
        model.modelSense = GRB.MINIMIZE
        model.setObjective(solution1 + solution2)

        '''优先级约束'''
        # 添加约束条件
        # constraint (2)
        model.addConstrs(sum(yf[i, j, v, q] for i in I_f_0 for v in V_f for q in Q_f) == 1 for j in I_f_vir)

        # constraint (3)
        model.addConstrs(sum(ys[i, j, v, q] for i in I_s_0 for v in V_s for q in Q_s) == 1 for j in I_s_vir)

        # constraint (4)
        model.addConstrs(sum(yf[i, j, v, q] for j in I_f_vir for v in V_f for q in Q_f) == 1 for i in I_f_0)

        # constraint (5)
        model.addConstrs(sum(ys[i, j, v, q] for j in I_s_vir for v in V_s for q in Q_s) == 1 for i in I_s_0)

        # constraint (6)
        model.addConstrs(sum(yf[i, -v, v, q] for i in (I_f + [current_task[v - 1]]) for q in Q_f) == 1 for v in V_f)

        # constraint (7)
        model.addConstrs(sum(ys[i, -v, v, q] for i in (I_s + [current_task[v - 1]]) for q in Q_s) == 1 for v in V_s)

        # constraint (8)
        model.addConstrs(sum(xf[i, v, p] for v in V_f for p in P_f) == 1 for i in I_f)

        # constraint (9)
        model.addConstrs(sum(xs[i, v, p] for v in V_s for p in P_s) == 1 for i in I_s)

        # constraint (10)
        model.addConstrs(sum(xf[-v, v, p] for p in P_f) == 1 for v in V_f)
        model.addConstrs(sum(xs[-v, v, p] for p in P_s) == 1 for v in V_s)

        # constraint (11)
        model.addConstrs(sum(xf[current_task[v - 1], v, p] for p in P_f) == 1 for v in V_f if len(current_not_virtual_task[v]) > 0)
        model.addConstrs(sum(xs[current_task[v - 1], v, p] for p in P_s) == 1 for v in V_s if len(current_not_virtual_task[v]) > 0)

        # constraint (11)
        model.addConstrs(sum(yf[current_task[v - 1], j, v, q] for j in (I_f + [-v]) for q in Q_f) == 1 for v in V_f)
        model.addConstrs(sum(ys[current_task[v - 1], j, v, q] for j in (I_s + [-v]) for q in Q_s) == 1 for v in V_s)

        # constraint (12)
        model.addConstrs(sum(z[r, v] for v in V_s) == 1 for r in R)

        # constraint (13)
        model.addConstrs(area[r, v] == sum(xs[i, v, p] for p in P_s for i in task_s_i[r]) for r in R for v in V_s)
        model.addConstrs(z[r, v] == min_(1, area[r, v]) for r in R for v in V_s)

        # constraint (14)
        model.addConstrs(
            sum(yf[i, j, v, q] for j in I_f_vir for q in Q_f) == sum(xf[i, v, p] for p in P_f) for i in I_f for v in
            V_f)

        # constraint (15)
        model.addConstrs(
            sum(yf[j, i, v, q] for j in I_f_0 for q in Q_f) == sum(xf[i, v, p] for p in P_f) for i in I_f for v in V_f)

        # constraint (16)
        model.addConstrs(
            sum(ys[i, j, v, q] for j in I_s_vir for q in Q_s) == sum(xs[i, v, p] for p in P_s) for i in I_s for v in
            V_s)

        # constraint (17)
        model.addConstrs(
            sum(ys[j, i, v, q] for j in I_s_0 for q in Q_s) == sum(xs[i, v, p] for p in P_s) for i in I_s for v in V_s)

        # constraint (19)
        model.addConstrs(sum(xf[i, v, p] for p in P_f for i in I_f) <= agv_max_task[v] for v in V_f)

        # constraint (20)
        model.addConstrs(sum(xs[i, v, p] for p in P_s for i in I_s) + agv_task[v] <= agv_max_task_num for v in V_s)

        model.setParam('OutputFlag', 0)
        model.optimize()
        solution = []
        if model.status == GRB.Status.INFEASIBLE:
            print('Optimization was stopped with status %d' % model.status)
            model.computeIIS()
            for c in model.getConstrs():
                if c.IISConstr:
                    print('%s' % c.constrName)
        for v in model.getVars():
            m = [v.varName, round(v.x)]
            solution.append(m)
        gurobi_solution = pd.DataFrame(solution, columns=("variable", "value"))
        gurobi_solution.to_csv(r"solution\{}_gurobi_solution.csv".format(time), index=False)

        task_dict = {}
        task_route = {}
        turn_point = [568, 579, 589, 601, 612, 622, 634, 645, 655, 667, 678, 688]
        for v in V_f:
            i = current_task[v - 1]
            task_dict[v] = [i]
            while True:
                flag = False
                for j in (I_f + [-v]):
                    for q in Q_f:
                        if int(gurobi_solution[gurobi_solution["variable"] == "yf_{}_{}_{}_{}".format(i, j, v, q)][
                                   "value"]) == 1:
                            task_dict[v].extend([j])
                            i = j
                            flag = True
                        if flag:
                            break
                    if flag:
                        break
                for p in P_f:
                    if int(gurobi_solution[gurobi_solution["variable"] == "xf_{}_{}_{}".format(i, v, p)]["value"]) == 1:
                        task_route[i] = pathf_i_v[i, v].Paths[p]
                if i < 0:
                    break
        for v in V_s:
            i = current_task[v - 1]
            task_dict[v] = [i]
            while True:
                flag = False
                for j in (I_s + [-v]):
                    for q in Q_s:
                        if int(gurobi_solution[gurobi_solution["variable"] == "ys_{}_{}_{}_{}".format(i, j, v, q)][
                                   "value"]) == 1:
                            task_dict[v].extend([j])
                            i = j
                            flag = True
                            break
                    if flag:
                        break
                for p in P_s:
                    if int(gurobi_solution[gurobi_solution["variable"] == "xs_{}_{}_{}".format(i, v, p)]["value"]) == 1:
                        task_route[i] = paths_i_v[i, v].Paths[p]
                if i < 0:
                    break
        for v in V_f:
            i = 1
            flag = False
            # 不更新当前执行的任务
            while True:
                m = []
                # 放货点到放货点出口
                if self.Task[task_dict[v][i - 1]].end != self.Map[self.Task[task_dict[v][i - 1]].end].entrance:
                    m += stock_entrance_path[
                        self.Task[task_dict[v][i - 1]].end, self.Map[self.Task[task_dict[v][i - 1]].end].entrance]
                    # 放货点出口到取货点出口
                    for q in Q_f:
                        if int(gurobi_solution[
                                   gurobi_solution["variable"] == "yf_{}_{}_{}_{}".format(task_dict[v][i - 1],
                                                                                          task_dict[v][i], v, q)][
                                   "value"]) == 1:
                            m += qf_i_j_v[task_dict[v][i - 1], task_dict[v][i], v].Paths[q][1:]
                else:
                    # 放货点出口到取货点出口
                    for q in Q_f:
                        if int(gurobi_solution[
                                   gurobi_solution["variable"] == "yf_{}_{}_{}_{}".format(task_dict[v][i - 1],
                                                                                          task_dict[v][i], v, q)][
                                   "value"]) == 1:
                            m += qf_i_j_v[task_dict[v][i - 1], task_dict[v][i], v].Paths[q]
                # 取货点出口到取货点
                if self.Map[self.Task[task_dict[v][i]].start].entrance != self.Task[task_dict[v][i]].start:
                    m += stock_entrance_path[
                             self.Map[self.Task[task_dict[v][i]].start].entrance, self.Task[task_dict[v][i]].start][1:]
                # 取货点到取货点出口
                if self.Task[task_dict[v][i]].start != self.Map[self.Task[task_dict[v][i]].start].entrance:
                    m += stock_entrance_path[
                             self.Task[task_dict[v][i]].start, self.Map[self.Task[task_dict[v][i]].start].entrance][1:]
                # 取货点出口到放货点出口
                if len(task_route[task_dict[v][i]]) > 1:
                    m += task_route[task_dict[v][i]][1:]
                '''对任务的终止节点进行更新'''
                if task_dict[v][i] > 0:
                    self.Task[task_dict[v][i]].end = candidate_area[v][i - 1]
                    if i > 1:
                        if candidate_area[v][i - 2] in turn_point:
                            flag = True
                    if flag:
                        m += [self.Map[candidate_area[v][i - 1]].entrance]
                # 放货点出口到放货点
                if self.Map[self.Task[task_dict[v][i]].end].entrance != self.Task[task_dict[v][i]].end:
                    m += stock_entrance_path[
                             self.Map[self.Task[task_dict[v][i]].end].entrance, self.Task[task_dict[v][i]].end][1:]
                self.Task[task_dict[v][i]].route_seq = m
                i += 1
                if i >= len(task_dict[v]):
                    break
        for v in V_s:
            i = 1
            # 不更新当前执行的任务
            while True:
                m = []
                # 放货点到放货点出口
                if self.Task[task_dict[v][i - 1]].end != self.Map[self.Task[task_dict[v][i - 1]].end].entrance:
                    m += stock_entrance_path[
                        self.Task[task_dict[v][i - 1]].end, self.Map[self.Task[task_dict[v][i - 1]].end].entrance]
                    # 放货点出口到取货点出口
                    for q in Q_s:
                        if int(gurobi_solution[
                                   gurobi_solution["variable"] == "ys_{}_{}_{}_{}".format(task_dict[v][i - 1],
                                                                                          task_dict[v][i], v, q)][
                                   "value"]) == 1:
                            m += qs_i_j_v[task_dict[v][i - 1], task_dict[v][i], v].Paths[q][1:]
                else:
                    # 放货点出口到取货点出口
                    for q in Q_s:
                        if int(gurobi_solution[
                                   gurobi_solution["variable"] == "ys_{}_{}_{}_{}".format(task_dict[v][i - 1],
                                                                                          task_dict[v][i], v, q)][
                                   "value"]) == 1:
                            m += qs_i_j_v[task_dict[v][i - 1], task_dict[v][i], v].Paths[q]
                # 取货点出口到取货点
                if self.Map[self.Task[task_dict[v][i]].start].entrance != self.Task[task_dict[v][i]].start:
                    m += stock_entrance_path[
                             self.Map[self.Task[task_dict[v][i]].start].entrance, self.Task[task_dict[v][i]].start][1:]
                # 取货点到取货点出口
                if self.Task[task_dict[v][i]].start != self.Map[self.Task[task_dict[v][i]].start].entrance:
                    m += stock_entrance_path[
                             self.Task[task_dict[v][i]].start, self.Map[self.Task[task_dict[v][i]].start].entrance][1:]
                # 取货点出口到放货点出口
                if len(task_route[task_dict[v][i]]) > 1:
                    m += task_route[task_dict[v][i]][1:]
                # 放货点出口到放货点
                if self.Map[self.Task[task_dict[v][i]].end].entrance != self.Task[task_dict[v][i]].end:
                    m += stock_entrance_path[
                             self.Map[self.Task[task_dict[v][i]].end].entrance, self.Task[task_dict[v][i]].end][1:]
                self.Task[task_dict[v][i]].route_seq = m
                i += 1
                if i >= len(task_dict[v]):
                    break
        for v in (V_s + V_f):
            flag = True
            for i in range(len(task_dict[v])):
                if task_dict[v][i] > 0:
                    flag = False
            if flag and current[v - 1] is None:
                task_dict[v] = []
            if flag and current[v - 1] == -v:
                task_dict[v] = [-v]
        for v in (V_s + V_f):
            if task_dict[v] and current[v - 1] is None:
                del task_dict[v][0]
        vehicle_flow = [1, 2, 3, 4]
        vehicle_stock = [5, 6, 7, 8]
        full_vehicle = []
        no_task_vehicle = []
        if len(V_f) < 4:
            for v in vehicle_flow:
                if v not in V_f:
                    full_vehicle.extend([v])
            for v in full_vehicle:
                task_dict[v] = self.AGV[v].tasklist
        if len(V_s) < 4:
            for v in vehicle_stock:
                if v not in V_s:
                    no_task_vehicle.extend([v])
            for v in no_task_vehicle:
                task_dict[v] = self.AGV[v].tasklist
        return task_dict
