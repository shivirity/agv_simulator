import copy
import colorlog
import logging

# set logger
log_colors_config = {
    'DEBUG': 'cyan',
    'INFO': 'green',
    'WARNING': 'yellow',
    'ERROR': 'red',
    'CRITICAL': 'bold_red',
}
logger = logging.getLogger("logger_rc")
logger.setLevel(logging.DEBUG)
sh = logging.StreamHandler()
sh.setLevel(logging.INFO)
stream_fmt = colorlog.ColoredFormatter(
    fmt="%(log_color)s[%(asctime)s] - %(filename)-8s - %(levelname)-7s - line %(lineno)s - %(message)s",
    log_colors=log_colors_config)
sh.setFormatter(stream_fmt)
logger.addHandler(sh)
sh.close()

NUM_OF_NODES = 829
NUM_OF_AGVS = 8


class RouteController:
    safe_grids = {
        0: list(range(664, 697)) + [363, 364],
        1: list(range(631, 664)) + [361, 362],
        2: list(range(598, 631)) + [357, 358, 359, 360],
        3: list(range(565, 598)) + [353, 354],
        4: list(range(697, 829)) + [63, 64, 65, 66] + list(range(437, 517)) + list(range(383, 413)),
        5: list(range(697, 829)) + [54, 55, 56, 57] + list(range(437, 517)) + list(range(383, 413)),
        6: list(range(697, 829)) + [133, 134, 135] + list(range(437, 517)) + list(range(383, 413)),
        7: list(range(697, 829)) + [138, 139, 140] + list(range(383, 413)) + list(range(437, 517)),
    }

    def __init__(self, routes: dict, size: list):
        """
        初始化

        :param routes: 每车规划路径
        """
        routes = copy.deepcopy(routes)
        # 全局常量
        self.num_of_nodes = NUM_OF_NODES  # 节点数量
        self.num_of_agv = NUM_OF_AGVS  # agv数量
        self.offline_task_num = size[0]
        self.online_task_num = size[1]

        # 路径规划相关
        self.residual_routes = routes  # 车辆剩余路径, dict

        # 全局变量
        self.reservation = None  # PR哈希, -1 -> 未预定, >=0 -> 被对应编号预定
        self.hash_route = [[] for _ in range(self.num_of_nodes + 1)]  # list
        self.update_shared_flag = [True for _ in range(self.num_of_agv)]
        self.update_route_free = False  # 路径更新后不再进行是否step的更新

        # 计算相关
        self.shared_routes = {agv: None for agv in range(self.num_of_agv)}  # dict
        self.shared_agvs = {agv: None for agv in range(self.num_of_agv)}  # dict

        # 初始化
        self.init_loc = [364, 362, 360, 354, 66, 57, 135, 140]
        self._init_hash_route(cur_loc=self.init_loc)
        self._init_shared_routes()

        # 随机因素
        self.online_first_rate = 0
        self.forward_access_prob = 0.05

    @staticmethod
    def _is_list_contained(lists, listl):
        """判断 lists 是否包含在 listl 中，是则返回 True"""
        lists, listl = list(lists), list(listl)
        assert 0 < len(lists) <= len(listl)
        for i in lists:
            if i not in listl:
                return False
        else:
            return True

    def _init_hash_route(self, cur_loc: list):
        """利用初始residual route初始化hash节点占据"""
        cur_loc = list(cur_loc)
        self._update_hash_route(cur_loc=cur_loc)

    def _update_hash_route(self, cur_loc: list):
        """车辆hash_route更新时调用"""
        cur_loc = list(cur_loc)
        self.hash_route = [[] for _ in range(self.num_of_nodes + 1)]
        for i in range(len(cur_loc)):
            if cur_loc[i] is None:
                continue
            if cur_loc[i] not in self.safe_grids[i]:
                self.hash_route[cur_loc[i]] = [i]
        for agv in range(self.num_of_agv):
            for grid in self.residual_routes[agv]:
                if grid in self.safe_grids[agv]:
                    break
                if not self.hash_route[grid]:
                    self.hash_route[grid] = [agv]
                else:
                    if agv not in self.hash_route[grid]:
                        self.hash_route[grid].append(agv)
                    assert self.hash_route[grid].count(agv) == 1

    def _init_shared_routes(self) -> None:
        """更新 residual route 后初始化连续 shared route 调用"""
        for agv in range(self.num_of_agv):
            _ = self._update_shared_routes(agv=agv)

    def _update_shared_routes(self, agv: int) -> bool:
        """更新共享路径，路径未更新时调用，返回shared_routes是否为空路径，为空False，不为空True"""
        for _grid in self.residual_routes[agv]:
            if _grid in self.safe_grids[agv]:
                self.shared_routes[agv] = []
                self.shared_agvs[agv] = []
                return False
            else:
                assert agv in self.hash_route[_grid]
                if len(self.hash_route[_grid]) > 1:
                    start_grid = _grid
                    break
        else:
            self.shared_routes[agv] = []
            self.shared_agvs[agv] = []
            return False

        # search from start_grid
        shared_routes, shared_agvs = [], []
        start_idx = self.residual_routes[agv].index(start_grid)
        for idx in range(start_idx, len(self.residual_routes[agv])):
            grid = self.residual_routes[agv][idx]
            if len(self.hash_route[grid]) > 1:
                shared_routes.append(grid)
                shared_agvs.append([item for item in self.hash_route[grid] if item != agv])
            else:
                assert len(self.hash_route[grid]) == 1 or grid in self.safe_grids[agv]
                break

        self.shared_routes[agv] = list(shared_routes)
        self.shared_agvs[agv] = copy.deepcopy(shared_agvs)
        return True

    def _update_reservation(self, loc_list: list) -> None:
        """利用当前位置更新预定，仅保留当前位置"""
        self.reservation = [-1 if i not in loc_list else loc_list.index(i) for i in range(self.num_of_nodes + 1)]

    def _is_shared_reserved(self, agv) -> bool:
        """
        判断共享区域是否被预定

        :param agv: 判断主体agv
        :return: 被其他车辆预定返回True，否则返回False
        """
        for grid in self.shared_routes[agv]:
            if self.reservation[grid] == agv:
                assert len([i for i in range(len(self.reservation)) if self.reservation[i] == agv]) == 1
                continue
            if self.reservation[grid] >= 0:
                return True
        else:
            return False

    def update_routes(self, routes: dict):
        """利用 new_routes 更新各车辆 residual route"""
        routes = copy.deepcopy(routes)
        assert isinstance(routes, dict)
        self.update_shared_flag = [True for _ in range(self.num_of_agv)]  # update允许更新列表
        loc_list = []
        # 更新 hash_route 和 residual_route
        for key, route in routes.items():
            self.residual_routes[key] = route[1:]  # update剩余路径
            loc_list.append(route[0] if len(route) else None)
            logger.debug(f'AGV{key} route updated.')
        # update hash routes
        self._init_hash_route(cur_loc=loc_list)
        # update shared routes
        self._init_shared_routes()
        self.update_route_free = True

    def get_instruction(self, status_list: list, loc_list: list, undo_offline_tasks: list,
                        step_list: list = None) -> list:
        """
        返回每辆车的控制策略（以列表形式）

        :param status_list: 每辆车的当前状态，布尔变量表示车辆当前是否可以前进，len=8
        :param loc_list: 当前所有车辆位置（节点编号），len=8
        :param undo_offline_tasks: 未完成的离线任务列表
        :param step_list: 表示在当前节点是否前进，用以更新 residual routes，True为已前进，False为未前进，len=8
        :return: 车辆控制策略列表
        """

        # 更新 reservation
        loc_list = list(loc_list)
        undo_offline_tasks = list(undo_offline_tasks)
        for loc in loc_list:
            assert loc is not None, f'{loc}'
        self._update_reservation(loc_list=loc_list)

        if step_list is not None and not self.update_route_free:
            # update residual_routes & hash_route
            step_grid_list = []  # 记录各车上一步路过节点，未前进则为-1
            # update residual_routes
            for agv in range(self.num_of_agv):
                if step_list[agv]:  # 前进节点
                    assert len(self.residual_routes[agv]) > 0
                    assert agv in self.hash_route[self.residual_routes[agv][0]] or \
                           self.residual_routes[agv][0] in self.safe_grids[agv], f'{self.hash_route[self.residual_routes[agv][0]]}'
                    step_grid = self.residual_routes[agv].pop(0)  # update residual_routes
                    step_grid_list.append(step_grid)
                else:  # 未前进节点
                    step_grid_list.append(-1)
            # update hash_route
            self._init_hash_route(cur_loc=loc_list)
            # update shared_routes & shared_agvs
            self._init_shared_routes()

        self.update_route_free = False

        control_list = []  # 控制指令列表
        # 对比潜在路径，更新权限申请情况
        online_first_flag = True if len(undo_offline_tasks) > (1 - self.online_first_rate) * self.offline_task_num else False
        # agv_order = list(range(self.num_of_agv)) if online_first_flag else [4, 5, 6, 7, 0, 1, 2, 3]
        agv_order = [0, 1, 2, 3, 5, 6, 4, 7] if online_first_flag else [5, 6, 4, 7, 0, 1, 2, 3]
        for agv in agv_order:
            if not status_list[agv]:  # agv无法行驶
                control_list.append(False)
            else:  # agv可以行驶
                if not len(self.residual_routes[agv]):
                    control_list.append(False)
                    # logging.warning(f'AGV{agv} has no residual route.')
                else:  # 车辆有剩余路径
                    next_step = self.residual_routes[agv][0]
                    # 权限申请条件1
                    if next_step not in self.shared_routes[agv] and self.reservation[next_step] < 0:
                        self.reservation[next_step] = agv
                        self.reservation[loc_list[agv]] = -1
                        control_list.append(True)
                    # 权限申请条件2
                    elif next_step in self.shared_routes[agv] and not self._is_shared_reserved(agv=agv):
                        self.reservation[next_step] = agv
                        self.reservation[loc_list[agv]] = -1
                        control_list.append(True)
                    else:  # 无法申请条件
                        control_list.append(False)
        control_list_new = [_ for _ in range(self.num_of_agv)]
        for i in range(len(agv_order)):
            control_list_new[agv_order[i]] = control_list[i]
        assert len(control_list) == self.num_of_agv
        return control_list_new
