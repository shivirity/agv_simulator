import copy
import colorlog
import logging
import random

random.seed(42)

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


class RouteController_basic_cor:

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
        self.hash_route = [[] for _ in range(self.num_of_nodes + 1)]  # list
        self.update_route_free = False  # 路径更新后不再进行是否step的更新

        # 初始化
        self._init_hash_route()

    def _init_hash_route(self):
        """初始化或重置"""
        self.hash_route = [[] for _ in range(self.num_of_nodes + 1)]  # list
        '''
        agv_list = [(agv, len(self.residual_routes[agv])) for agv in range(self.num_of_agv)]
        agv_list = sorted(agv_list, key=lambda x: x[1], reverse=False)  # 升序
        agv_list = [i[0] for i in agv_list]
        '''
        agv_list = [i for i in range(self.num_of_agv)]
        for agv in agv_list:
            self._update_hash_route(agv=agv)

    def _update_hash_route(self, agv: int):
        """更新单车hash_route"""
        for grid in self.residual_routes[agv]:
            if not self.hash_route[grid]:
                self.hash_route[grid] = [agv]
            else:
                self.hash_route[grid].append(agv)

    def update_routes(self, routes: dict):
        """利用 new_routes 更新各车辆 residual route"""
        assert isinstance(routes, dict)
        routes = copy.deepcopy(routes)
        for key, route in routes.items():
            self.residual_routes[key] = route  # update剩余路径
        self._init_hash_route()  # update hash_route
        self.update_route_free = True
        logger.debug(f'hash_route updated.')

    def get_instruction(self, status_list: list, loc_list: list, step_list: list = None) -> list:
        """
        返回每辆车的控制策略

        :param status_list: 每辆车的当前状态，布尔变量表示车辆当前是否可以前进，len=8
        :param loc_list: 当前所有车辆位置（节点编号），len=8
        :param step_list: 表示在当前节点是否前进，用以更新 residual routes，True为已前进，False为未前进，len=8
        :return: 车辆控制策略列表
        """

        if step_list is not None and not self.update_route_free:  # 非初始化
            # update residual route & hash route
            step_grid_list = []  # 记录各车上一步路过节点，未前进则为-1
            for agv in range(self.num_of_agv):
                if step_list[agv]:  # 前进节点
                    # update residual route
                    assert len(self.residual_routes[agv]) > 0
                    assert agv in self.hash_route[
                        self.residual_routes[agv][0]], f'{self.hash_route[self.residual_routes[agv][0]]}'
                    step_grid = self.residual_routes[agv].pop(0)  # update residual route
                    step_grid_list.append(step_grid)
                    # update hash route
                    assert self.hash_route[step_grid][0] == agv
                    self.hash_route[step_grid].pop(0)  # cancel reservation
                else:  # 未前进节点
                    step_grid_list.append(-1)
        self.update_route_free = False

        control_list = []
        # forall agv, update get control instruction
        for agv in range(self.num_of_agv):
            if not status_list[agv]:  # agv无法行驶
                control_list.append(False)
            else:  # agv可以行驶
                if not len(self.residual_routes[agv]):  # has no residual route
                    control_list.append(False)
                else:  # has residual route
                    next_step = self.residual_routes[agv][0]
                    # 权限申请
                    assert self.hash_route[next_step]
                    if self.hash_route[next_step][0] == agv:
                        control_list.append(True)
                    else:
                        control_list.append(False)

        assert len(control_list) == self.num_of_agv
        return control_list