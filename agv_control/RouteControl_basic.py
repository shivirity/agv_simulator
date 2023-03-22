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


class RouteController_basic:

    safe_grids = {
        0: list(range(664, 697)) + [364],
        1: list(range(631, 664)) + [362],
        2: list(range(598, 631)) + [360],
        3: list(range(565, 598)) + [354],
        4: list(range(697, 730)) + [66] + list(range(437, 462)),
        5: list(range(730, 763)) + [57] + list(range(462, 487)),
        6: list(range(763, 796)) + [135] + list(range(487, 517)),
        7: list(range(796, 829)) + [140] + list(range(383, 413)),
    }

    wait = 20  # 等待side collision

    def __init__(self, routes: dict):
        """
        初始化

        :param routes: 每车规划路径
        """
        routes = copy.deepcopy(routes)
        # 全局常量
        self.num_of_nodes = NUM_OF_NODES  # 节点数量
        self.num_of_agv = NUM_OF_AGVS  # agv数量

        # 路径规划相关
        self.residual_routes = routes  # 车辆剩余路径, dict

        # 全局变量
        self.reservation = None  # PR哈希, -1 -> 未预定, >=0 -> 被对应编号预定
        self.hash_route = [[] for _ in range(self.num_of_nodes+1)]  # list
        self.hash_prior = [[] for _ in range(self.num_of_nodes+1)]  # list
        self.update_route_free = False  # 路径更新后不再进行是否step的更新
        self.wait_coll = [0 for _ in range(self.num_of_agv)]  # 等待side collision

        # 存储变量
        self.last_locs = [[] for _ in range(self.num_of_agv)]  # list

        # 初始化
        self._init_hash_route()

    @property
    def exist_prior(self) -> list:
        return list(set([i for j in self.hash_prior for i in j]))

    def _init_hash_route(self):
        """利用初始residual route初始化hash节点占据"""
        for agv in range(self.num_of_agv):
            self._update_hash_route(agv=agv)

    def _update_hash_route(self, agv: int, former_route: list = None):
        """
        车辆hash_route更新时调用

        :param agv: 待更新的车辆编号
        :param former_route: 车辆原来路径，为None代表系统初始化
        :return:
        """
        # clear former route
        if former_route is not None:
            former_route = list(former_route)
            for grid in former_route:
                if agv in self.hash_route[grid]:
                    self.hash_route[grid].remove(agv)
            # add new route
            for grid in self.residual_routes[agv]:
                if not self.hash_route[grid]:
                    self.hash_route[grid] = [agv]
                if agv not in self.hash_route[grid]:
                    self.hash_route[grid].append(agv)
                assert self.hash_route[grid].count(agv) == 1
        else:
            for grid in self.residual_routes[agv]:
                if not self.hash_route[grid]:
                    self.hash_route[grid] = [agv]
                else:
                    if agv not in self.hash_route[grid]:
                        self.hash_route[grid].append(agv)
                    assert self.hash_route[grid].count(agv) == 1

    def _update_hash_prior(self, agv):
        """车辆进入路径节点时更新路径节点的预约列表，优先级置于最后"""
        for grid in self.residual_routes[agv]:
            if grid in self.safe_grids[agv]:
                break
            else:
                self.hash_prior[grid].append(agv)
        else:
            assert False

    def _get_last_loc(self, agv, cur_loc) -> int:
        """返回agv的上一个位置，cur_loc代表当前位置"""
        for i in range(len(self.last_locs[agv])):
            poten_loc = self.last_locs[agv][len(self.last_locs[agv])-1-i]
            poten_loc = int(poten_loc)
            if poten_loc != cur_loc:
                return poten_loc
        else:
            return -1

    def _update_reservation(self, loc_list: list) -> None:
        """利用当前位置更新预定，仅保留当前位置"""
        self.reservation = [-1 if i not in loc_list else loc_list.index(i) for i in range(self.num_of_nodes+1)]

    def update_routes(self, routes: dict):
        """利用 new_routes 更新各车辆 residual route"""
        routes = copy.deepcopy(routes)
        assert isinstance(routes, dict)
        for key, route in routes.items():
            former_route = list(self.residual_routes[key])
            self.residual_routes[key] = route  # update剩余路径
            self._update_hash_route(agv=key, former_route=former_route)  # update路径hash
            logger.debug(f'AGV{key} route updated.')
        self.update_route_free = True

    def get_instruction(self, status_list: list, loc_list: list, undo_offline_tasks: list, step_list: list = None) -> list:
        """
        返回每辆车的控制策略（以列表形式）

        :param status_list: 每辆车的当前状态，布尔变量表示车辆当前是否可以前进，len=8
        :param loc_list: 当前所有车辆位置（节点编号），len=8
        :param undo_offline_tasks: 未完成的离线任务列表
        :param step_list: 表示在当前节点是否前进，用以更新 residual routes，True为已前进，False为未前进，len=8
        :return: 车辆控制策略列表
        """
        loc_list = list(loc_list)
        undo_offline_tasks = list(undo_offline_tasks)
        for loc in loc_list:
            assert loc is not None, f'{loc}'
        for agv in range(self.num_of_agv):
            self.last_locs[agv].append(loc_list[agv])
        self._update_reservation(loc_list=loc_list)
        last_locs = [self._get_last_loc(i, loc_list[i]) for i in range(self.num_of_agv)]

        if step_list is not None and not self.update_route_free:
            # update residual_routes & hash_route
            step_grid_list = []  # 记录各车上一步路过节点，未前进则为-1
            for agv in range(self.num_of_agv):
                if step_list[agv]:  # 前进节点
                    assert len(self.residual_routes[agv]) > 0
                    assert agv in self.hash_route[self.residual_routes[agv][0]], f'{self.hash_route[self.residual_routes[agv][0]]}'
                    step_grid = self.residual_routes[agv].pop(0)  # update residual_routes
                    assert step_grid == loc_list[agv]
                    step_grid_list.append(step_grid)
                    if step_grid not in self.residual_routes[agv]:
                        self.hash_route[step_grid].remove(agv)  # update hash_route
                else:  # 未前进节点
                    step_grid_list.append(-1)
        self.update_route_free = False

        # update prior
        for agv in range(self.num_of_agv):
            if step_list is None or step_list[agv] is False:
                continue
            else:
                # grid -> safe
                if last_locs[agv] not in self.safe_grids[agv] and loc_list[agv] in self.safe_grids[agv]:
                    assert self.wait_coll[agv] == 0
                    self.wait_coll[agv] += self.wait
                    assert agv not in self.exist_prior
                # grid -> grid
                elif last_locs[agv] not in self.safe_grids[agv] and loc_list[agv] not in self.safe_grids[agv]:
                    assert self.hash_prior[loc_list[agv]][0] == agv
                    self.hash_prior[loc_list[agv]].pop(0)
                # safe -> grid
                elif last_locs[agv] in self.safe_grids[agv] and loc_list[agv] not in self.safe_grids[agv]:
                    assert self.hash_prior[loc_list[agv]][0] == agv
                    self.hash_prior[loc_list[agv]].pop(0)
                # safe -> safe
                else:
                    assert last_locs[agv] in self.safe_grids[agv] and loc_list[agv] in self.safe_grids[agv]

        control_list = []  # 控制指令列表
        for agv in range(self.num_of_agv):
            if not status_list[agv]:  # agv无法行驶
                control_list.append(False)
            else:  # agv可以行驶
                if not len(self.residual_routes[agv]):
                    control_list.append(False)
                else:  # 车辆有剩余路径
                    next_step = self.residual_routes[agv][0]

                    # update hash prior
                    if next_step not in self.safe_grids[agv] and loc_list[agv] in self.safe_grids[agv] and \
                            agv not in self.exist_prior:
                        self._update_hash_prior(agv=agv)

                    if next_step in self.safe_grids[agv]:
                        if self.wait_coll[agv] > 0:
                            self.wait_coll[agv] -= 1
                            control_list.append(False)
                        else:
                            self.reservation[next_step] = agv
                            self.reservation[loc_list[agv]] = -1
                            control_list.append(True)
                    else:
                        if self.hash_prior[next_step][0] == agv and self.reservation[next_step] < 0:
                            self.reservation[next_step] = agv
                            self.reservation[loc_list[agv]] = -1
                            control_list.append(True)
                        else:
                            control_list.append(False)

        assert len(control_list) == self.num_of_agv
        return control_list
