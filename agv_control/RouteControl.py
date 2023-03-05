import copy
import logging

NUM_OF_NODES = 829
NUM_OF_AGVS = 8


class RouteController:
    def __init__(self, routes: dict):
        """
        初始化

        :param routes: 每车规划路径
        """
        # 全局常量
        self.num_of_nodes = NUM_OF_NODES  # map路径信息
        self.num_of_agv = NUM_OF_AGVS  # agv数量

        # 路径规划相关
        self.residual_routes = routes  # 车辆规划路径（包含转弯耗时）, dict

        # 全局变量
        self.reservation = None  # PR哈希, -1 -> 未预定, >=0 -> 被对应编号预定
        self.hash_route = [0 for _ in range(self.num_of_nodes)]  # list
        self.update_shared_flag = [True for _ in range(self.num_of_agv)]

        # 计算相关
        # self.total_shared_routes = self._init_total_shared_routes()  # dict
        self.shared_routes = {agv: None for agv in range(self.num_of_agv)}  # dict
        self.shared_agvs = {agv: None for agv in range(self.num_of_agv)}  # dict

        # 初始化
        self._init_hash_route()
        self._init_shared_routes()

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

    def _init_hash_route(self):
        """利用初始residual route初始化hash节点占据"""
        for agv in range(self.num_of_agv):
            self._update_hash_route(agv=agv)

    def _update_hash_route(self, agv: int, former_route: list = None):
        """
        车辆residual_route更新时调用

        :param agv: 待更新的车辆编号
        :param former_route: 车辆原来路径，为None代表系统初始化
        :return:
        """
        # clear former route
        if former_route is not None:
            former_route = list(former_route)
            for grid in former_route:
                self.hash_route[grid].remove(agv)
            # add new route
            for grid in self.residual_routes[agv]:
                if agv not in self.hash_route[grid]:
                    self.hash_route[grid].append(agv)
                assert self.hash_route[grid].count(agv) == 1
        else:
            for grid in self.residual_routes[agv]:
                if self.hash_route[grid] == 0:
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
        for grid in self.residual_routes[agv]:
            if len(self.hash_route[grid]) > 1:
                start_grid = grid
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
                if grid not in shared_routes:
                    shared_routes.append(grid)
                    shared_agvs.append([item for item in self.hash_route[grid] if item != agv])
            else:
                break

        self.shared_routes[agv] = list(shared_routes)
        self.shared_agvs[agv] = copy.deepcopy(shared_agvs)
        return True

    def _update_reservation(self, loc_list: list) -> None:
        """利用当前位置更新预定，仅保留当前位置"""
        self.reservation = [-1 if i not in loc_list else loc_list.index(i) for i in range(self.num_of_nodes)]

    def _is_shared_reserved(self, agv) -> bool:
        """
        判断共享区域是否被预定

        :param agv: 判断主体agv
        :return: 被其他车辆预定返回True，否则返回False
        """
        for grid in self.shared_routes[agv]:
            assert self.reservation[grid] != agv
            if self.reservation[grid] >= 0:
                return True
        else:
            return False

    def update_routes(self, routes: dict):
        routes = dict(routes)
        """利用 new_routes 更新各车辆 residual route"""
        assert isinstance(routes, dict)
        self.update_shared_flag = [True for _ in range(self.num_of_agv)]  # update允许更新列表
        for key, route in routes.items():
            former_route = list(self.residual_routes[key])
            self.residual_routes[key] = route  # update剩余路径
            self._update_hash_route(agv=key, former_route=former_route)  # update路径hash
            logging.warning(f'AGV{key} route updated.')
        # 更新 hash_route 和 residual_route
        self._init_shared_routes()

    def get_instruction(self, status_list: list, loc_list: list, step_list: list = None) -> list:
        """
        返回每辆车的控制策略（以列表形式）

        :param status_list: 每辆车的当前状态，布尔变量表示车辆当前是否可以前进，len=8
        :param loc_list: 当前所有车辆位置（节点编号），len=8
        :param step_list: 表示在当前节点是否前进，用以更新 residual routes，True为已前进，False为未前进，len=8
        :return: 车辆控制策略列表
        """

        # 更新 reservation
        loc_list = list(loc_list)
        self._update_reservation(loc_list=loc_list)

        if step_list is not None:
            # update residual_routes & hash_route
            step_grid_list = []  # 记录各车上一步路过节点，未前进则为-1
            for agv in range(self.num_of_agv):
                if step_list[agv]:  # 前进节点
                    assert len(self.residual_routes[agv]) > 0
                    step_grid = self.residual_routes[agv].pop(0)  # update residual_routes
                    step_grid_list.append(step_grid)
                    self.hash_route[step_grid].remove(agv)  # update hash_route
                else:  # 未前进节点
                    step_grid_list.append(-1)

            # update shared_routes & shared_agvs
            for agv in range(self.num_of_agv):
                # 没有剩余路径，需要更新或指示任务完成
                if len(self.residual_routes[agv]) == 0:
                    logging.warning(f'AGV{agv} needs to update residual route.')
                # 有剩余路径
                else:
                    # 如果前进，则删去shared route里的对应元素（如果存在）
                    if step_list[agv]:
                        if step_grid_list[agv] in self.shared_routes[agv]:
                            del_idx = self.shared_routes[agv].index(step_grid_list[agv])
                            assert del_idx == 0
                            self.shared_routes[agv].pop(del_idx)
                            self.shared_agvs[agv].pop(del_idx)
                    # check 其他车辆前进
                    for idx in range(len(self.shared_routes[agv])):
                        # update due to moving forward
                        if self._is_list_contained(
                                lists=self.shared_agvs[agv][idx],
                                listl=self.hash_route[self.shared_routes[agv][idx]]):
                            assert len(self.hash_route[self.shared_routes[agv][idx]]) == len(
                                self.shared_agvs[agv][idx]) + 1
                        else:
                            # delete
                            if len(self.hash_route[self.shared_routes[agv][idx]]) == 1:
                                self.shared_routes[agv] = self.shared_routes[agv][:idx]
                                self.shared_agvs[agv] = self.shared_agvs[agv][:idx]
                            # update grid
                            else:
                                for ano_agv in self.shared_agvs[agv][idx]:
                                    if ano_agv not in self.hash_route[self.shared_routes[agv][idx]]:
                                        self.shared_agvs[agv][idx].remove(ano_agv)

                # 若 shared_route 为空，则更新下一段最新的
                if len(self.shared_routes[agv]) == 0 and self.update_shared_flag[agv]:
                    flag = self._update_shared_routes(agv=agv)
                    if not flag:
                        self.update_shared_flag[agv] = flag

        control_list = []  # 控制指令列表
        # 对比潜在路径，更新权限申请情况
        for agv in range(self.num_of_agv):
            if not status_list[agv]:  # agv无法行驶
                control_list.append(False)
            else:  # agv可以行驶
                if not len(self.residual_routes[agv]):
                    control_list.append(False)
                    logging.warning(f'AGV{agv} has no residual route.')
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

        assert len(control_list) == self.num_of_agv
        return control_list
