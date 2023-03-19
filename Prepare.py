import random
import numpy as np
import pandas as pd
import logging
import colorlog

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
        self.route_scheduling = None

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
                    assert len(self.AGV[i+1].route) == 2
                    assert not self.AGV[i+1].tasklist

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
                if i <= 3 and self.Task[self.AGV[i+1].task].end == self.Task_order[i+1][0]:
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
                    assert len(self.AGV[i].route) == 0, f'agv{i}'
                    self.AGV[i].task = self.AGV[i].tasklist[0]
                    self.AGV[i].task_status = 'get'
                    self.Map[self.Task[self.AGV[i].task].start].reservation = True
                    self.Map[self.Task[self.AGV[i].task].end].reservation = True
                    for j in self.AGV[i].tasklist:
                        if j == self.AGV[i].task:
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
            )
        else:
            self.instruction = self.controller.get_instruction(
                status_list=[True if i < 0.1 else False for i in self.move_status],
                loc_list=[self.AGV[i].location for i in range(1, 9)],
                step_list=self.moving_success
            )

        for agv in range(8):
            if len(self.AGV[agv+1].route) >= 3:
                assert self.AGV[agv+1].route[2] == self.controller.residual_routes[agv][0]

        # 下发控制路径 step_list 上一步到底有没有走成功，转弯不算走，移动

        self.error_generating()  # 误差生成

        # 冲突检测+策略修正
        self.fix_instruction()
        self.agvs_move(self.instruction)
        self.agvs_status_change()
        self.update_step()
        self.finish_identify()
        self.deadlock_happen = False  # reset deadlock


