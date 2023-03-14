import copy
from queue import PriorityQueue
from collections import deque


# 树结构，用于A-star回溯路径
class NodeVector:
    node = None  # 当前的节点
    frontNode = None  # 当前节点的前置节点
    childNodes = None  # 当前节点的后置节点们
    g = 0  # 起点到当前节点所经过的距离
    h = 0  # 启发值

    def __init__(self, node):
        self.node = node
        self.childNodes = []

    @property
    def f(self):
        return self.g + self.h

    def calcGH(self, target):
        """target代表目标节点，Grid类"""
        self.g = self.frontNode.g + \
                 abs(self.node.x - self.frontNode.node.x) + \
                 abs(self.node.y - self.frontNode.node.y)
        dx = abs(target.x - self.node.x)
        dy = abs(target.y - self.node.y)
        self.h = dx + dy

    def __lt__(self, other):
        return self.f < other.f


class A_star:
    def __init__(self, grids, start_point, end_point):
        self.grids = grids
        self.start_point = start_point
        self.end_point = end_point

        self.open_set = PriorityQueue()
        self.closed_set = deque()

        self.found_end_node = None  # 寻找到的终点，用于判断算法结束

    def is_closed(self, grid_id):
        """判断id节点是否在closed_set中，在返回True，不在返回False"""
        return grid_id in [vector.node.id for vector in self.closed_set]

    def get_route(self):
        """输出最优路径"""
        route = [self.found_end_node.node.id]
        current = self.found_end_node
        while True:
            current = current.frontNode
            route.append(current.node.id)
            if current.node.id == self.start_point.id:
                break
        return list(reversed(route))

    def process(self):
        # 初始化open集合，并把起始点放入
        self.open_set.put((0, NodeVector(self.start_point)))

        # 开始迭代，直到找到终点，或找完了所有能找的点
        while self.found_end_node is None and not self.open_set.empty():
            # 选出下一个合适的点
            vector = self.popLowGHNode()  # vector: NodeVector
            # 获取合适点周围所有的邻居
            # todo 这里的neighbor是索引
            neighbors = [self.grids[i] for i in vector.node.neighbor if not self.is_closed(i)]
            for neighbor in neighbors:
                # 初始化邻居，并计算g和h
                child = NodeVector(neighbor)
                child.frontNode = vector
                child.calcGH(self.end_point)
                vector.childNodes.append(child)

                # 添加到open集合中
                assert isinstance(child.f, float), f'{child.f}'
                # self.open_set.put((child, child.f))
                self.open_set.put([child.f, child])

                # 找到终点
                if neighbor == self.end_point:
                    self.found_end_node = child

        if self.found_end_node is None:
            logging.warning(f'无法找到从{self.start_point.id}到{self.end_point.id}的路')
            return None
        else:
            route = self.get_route()
            return route

    # A*，寻找f = g + h最小的节点
    def popLowGHNode(self):
        found_node = self.open_set.get()[1]
        self.closed_set.append(found_node)
        return found_node


class Routeplanner_basic:
    park_list = [364, 362, 360, 354, 66, 57, 135, 140]  # agv停靠点列表
    car2set = {
        1: 1,
        2: 2,
        3: 3,
        4: 4
    }

    def __init__(self, grids: dict):
        self.grids = grids

    def get_route(self, start: int, end: int):
        return A_star(grids=self.grids, start_point=self.grids[start], end_point=self.grids[end]).process()

    # 0~3: 在线，4~7: 离线
    # type 1
    def route_scheduling(
            self,
            agv_loc: list,  # 车辆当前位置，节点编号
            agv_task_list: list,  # 车辆当前任务列表
            agv_status_list: list,  # 任务执行状态（要去xx）（'get', 'put', 'return'）
            task_to_plan: list,  # 待规划的任务，任务编号列表
            task_dict: dict,  # 任务池，包含已经执行的任务和待规划的任务，任务信息（取货点、卸货点）
            outer_dict: dict  # 各个车辆的待选库位列表（dict）
    ) -> dict:
        """
        ** put和get需要分成路径中和执行等待中考虑
        ** new situations
        for agv in range(8):
            # 区分任务种类
            if 在线:
                # check 车辆状态
                if 没有当前任务:
                    if 后续有任务:
                        任务列表更新为[next1, next2, return]
                        Task[next1], Task[next2], return 更新start和end
                    else 后续没有任务:
                        do nothing
                if 只有一个当前任务在执行:
                    assert task_list[0] == '-agv';状态=='return'(只有一个返回任务);
                    if 后续有任务:
                        到当前return点，并去下一个任务取货点，规划后至多两个任务（列表里读取最外面两个库位），最后加入返回的0任务
                        任务列表更新为[current(return), next1, next2, return]
                        Task[next1], Task[next2], return 更新start和end
                    else 后续没有任务:
                        继续执行完，到停靠点等待
                        任务列表更新为[current(return)]
                        Task 不需要更新
                elif 有两个任务在执行:
                    assert 当前在'get' or 'put'
                        if 后续有任务:
                            继续执行完（到当前put点），并去下一个任务取货点继续，最后规划0任务
                            任务列表 - [current, next1, return]
                            Task[next1], return 更新start和end
                        else 后续没有任务:
                            继续执行完，到停靠点等待
                            任务列表不需要更新
                            Task 不需要更新
                elif 有三个任务在执行:
                    if current is return:
                        # [-agv, 1, -agv]
                        if 后续有任务:
                            继续执行完（到第一个return的停靠点），并去规划后两个任务（[1, 2]），最后规划0任务
                            任务列表 - [current(return), 1, 2, return]
                            Task update 1 and 2 (maybe not including 1)
                        else 后续无任务:
                            不需更新
                    else current is not return:
                        # [1, 2, -agv]
                        if 后续有任务:
                            继续执行完（到第一个任务的卸货点），并去规划后至多一个任务（[2]），最后规划0任务
                            任务列表 - baseline里不更新了
                            不更新
                        else 后续无任务:
                            不需更新
                elif 有四个任务在执行:
                    assert current is return
                    # [-agv, 1, 2, -agv]
                    if 后续有任务:
                        不更新
                    else 后续没有任务:
                        不更新
            else 离线:
                if 当前只有一个任务在执行:
        """
        outer_dict = copy.deepcopy(outer_dict)
        for key in outer_dict.keys():
            outer_dict[key] = outer_dict[key][:2]

        agv_status_list, task_to_plan = list(agv_status_list), list(task_to_plan)
        agv_task_list = copy.deepcopy(agv_task_list)

        # element check
        for task_list in agv_task_list:
            assert isinstance(task_list, list)

        cur_task_list = [i[0] for i in agv_task_list if len(i) > 0]
        offline_tasks = [i for i in task_to_plan if i < 101]
        online_tasks = list(sorted([i for i in task_to_plan if i > 100]))
        online_tasks = [i for i in online_tasks if i not in cur_task_list]

        # 离线任务和车对应的字典
        agv2offlinetask = {
            5: [i for i in offline_tasks if 1 <= i < 26],
            6: [i for i in offline_tasks if 26 <= i < 51],
            7: [i for i in offline_tasks if 51 <= i < 76],
            8: [i for i in offline_tasks if 76 <= i < 101],
        }
        # 任务列表字典，路线列表字典
        new_task_dict = {i: None for i in range(1, 9)}

        for agv in range(8):
            if agv < 4:  # 在线任务
                if len(agv_task_list[agv]) == 0 or len(agv_task_list[agv]) == 1:  # 没有当前任务或只有一个任务在执行（return）
                    if len(agv_task_list[agv]) == 0:
                        assert agv_loc[agv] == self.park_list[
                            agv], f'agv{agv + 1}在任务列表长度为0时位置错误，cur_loc={agv_loc[agv]}'
                    else:
                        assert agv_task_list[agv][0] == -(agv + 1) and agv_status_list[
                            agv] == 'return', f'agv{agv}, task_list_len = 1'
                    if online_tasks:  # 后续有任务
                        if len(online_tasks) - 2 >= (0 if agv == 3 else 1):  # 分配两个任务
                            if len(outer_dict[self.car2set[agv + 1]]) == 2:  # 剩余两个以上库位
                                assign_task_1 = online_tasks.pop(0)  # 1st待分配的任务id
                                assign_task_2 = online_tasks.pop(0)  # 2nd待分配的任务id
                                end_1, end_2 = (outer_dict[self.car2set[agv + 1]][i] for i in range(2))
                                task_dict[assign_task_1].end, task_dict[assign_task_2].end = end_1, end_2
                                task_1_get = self.get_route(start=self.park_list[agv],
                                                            end=task_dict[assign_task_1].start)
                                task_1_put = self.get_route(start=task_dict[assign_task_1].start,
                                                            end=task_dict[assign_task_1].end)
                                task_2_get = self.get_route(start=task_dict[assign_task_1].end,
                                                            end=task_dict[assign_task_2].start)
                                task_2_put = self.get_route(start=task_dict[assign_task_2].start,
                                                            end=task_dict[assign_task_2].end)
                                task_return = self.get_route(start=task_dict[assign_task_2].end,
                                                             end=self.park_list[agv])
                                # new task list
                                if len(agv_task_list[agv]) == 0:
                                    new_task_dict[agv + 1] = [assign_task_1, assign_task_2, -int(agv + 1)]
                                else:
                                    new_task_dict[agv + 1] = [-int(agv + 1), assign_task_1, assign_task_2,
                                                              -int(agv + 1)]
                                # change task_dict
                                task_dict[assign_task_1].route_seq = list(task_1_get) + list(task_1_put[1:])
                                task_dict[assign_task_2].route_seq = list(task_2_get) + list(task_2_put[1:])
                                task_dict[-int(agv+1)].route_seq = list(task_return)
                            elif len(outer_dict[self.car2set[agv + 1]]) == 1:  # 剩余一个库位
                                assign_task = online_tasks.pop(0)  # 待分配的任务id
                                end = (outer_dict[self.car2set[agv + 1]][0])
                                task_dict[assign_task].end = end
                                task_get = self.get_route(start=self.park_list[agv],
                                                          end=task_dict[assign_task].start)
                                task_put = self.get_route(start=task_dict[assign_task].start,
                                                          end=task_dict[assign_task].end)
                                task_return = self.get_route(start=task_dict[assign_task].end, end=self.park_list[agv])
                                # new task list
                                if len(agv_task_list[agv]) == 0:
                                    new_task_dict[agv + 1] = [assign_task, -int(agv + 1)]
                                else:
                                    new_task_dict[agv + 1] = [-int(agv + 1), assign_task, -int(agv + 1)]
                                # change task_dict
                                task_dict[assign_task].route_seq = list(task_get) + list(task_put[1:])
                                task_dict[-int(agv+1)].route_seq = list(task_return)
                            else:  # 没有剩余库位
                                new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out
                        else:  # 分配一个在线任务
                            if len(outer_dict[self.car2set[agv + 1]]) >= 1:  # 剩余一个库位
                                assign_task = online_tasks.pop(0)  # 待分配的任务id
                                end = (outer_dict[self.car2set[agv + 1]][0])
                                task_dict[assign_task].end = end
                                task_get = self.get_route(start=self.park_list[agv],
                                                          end=task_dict[assign_task].start)
                                task_put = self.get_route(start=task_dict[assign_task].start,
                                                          end=task_dict[assign_task].end)
                                task_return = self.get_route(start=task_dict[assign_task].end, end=self.park_list[agv])
                                # new task list
                                if len(agv_task_list[agv]) == 0:
                                    new_task_dict[agv + 1] = [assign_task, -int(agv + 1)]
                                else:
                                    new_task_dict[agv + 1] = [-int(agv + 1), assign_task, -int(agv + 1)]
                                # change task_dict
                                task_dict[assign_task].route_seq = list(task_get) + list(task_put[1:])
                                task_dict[-int(agv+1)].route_seq = list(task_return)
                            else:
                                new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out
                    else:  # 没有在线任务
                        new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out
                elif len(agv_task_list[agv]) == 2:  # 只有一个当前任务在执行
                    assert agv_status_list[agv] in ('get', 'put')
                    if len(online_tasks) >= 1 and len(outer_dict[self.car2set[agv + 1]]) >= 1:  # 指派一个任务
                        exec_task = agv_task_list[agv][0]
                        assign_task = online_tasks.pop(0)  # 待分配的任务id
                        end = (outer_dict[self.car2set[agv + 1]][0])
                        task_dict[assign_task].end = end
                        task_get = self.get_route(start=task_dict[exec_task].end,
                                                  end=task_dict[assign_task].start)
                        task_put = self.get_route(start=task_dict[assign_task].start,
                                                  end=task_dict[assign_task].end)
                        task_return = self.get_route(start=task_dict[assign_task].end, end=self.park_list[agv])
                        # new task list
                        new_task_dict[agv + 1] = [exec_task, assign_task, -int(agv + 1)]
                        # change task_dict
                        task_dict[assign_task].route_seq = list(task_get) + list(task_put[1:])
                        task_dict[-int(agv+1)].route_seq = list(task_return)
                    else:  # 不需要指派任务
                        new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out
                elif len(agv_task_list[agv]) == 3:  # 当前三个任务在列表中
                    if agv_status_list[agv] == 'return':  # [-agv, 1, -agv]
                        # 指派两个任务，取3保证一定宽放量，避免有车空闲却无任务可接
                        if len(online_tasks) >= 3 and len(outer_dict[self.car2set[agv + 1]]) >= 2:
                            assign_task_1 = online_tasks.pop(0)  # 1st待分配的任务id
                            assign_task_2 = online_tasks.pop(0)  # 2nd待分配的任务id
                            end_1, end_2 = (outer_dict[self.car2set[agv + 1]][i] for i in range(2))
                            task_dict[assign_task_1].end, task_dict[assign_task_2].end = end_1, end_2
                            task_1_get = self.get_route(start=self.park_list[agv],
                                                        end=task_dict[assign_task_1].start)
                            task_1_put = self.get_route(start=task_dict[assign_task_1].start,
                                                        end=task_dict[assign_task_1].end)
                            task_2_get = self.get_route(start=task_dict[assign_task_1].end,
                                                        end=task_dict[assign_task_2].start)
                            task_2_put = self.get_route(start=task_dict[assign_task_2].start,
                                                        end=task_dict[assign_task_2].end)
                            task_return = self.get_route(start=task_dict[assign_task_2].end,
                                                         end=self.park_list[agv])
                            # new task list
                            new_task_dict[agv + 1] = [-int(agv + 1), assign_task_1, assign_task_2, -int(agv + 1)]
                            # change task_dict
                            task_dict[assign_task_1].route_seq = list(task_1_get) + list(task_1_put[1:])
                            task_dict[assign_task_2].route_seq = list(task_2_get) + list(task_2_put[1:])
                            task_dict[-int(agv+1)].route_seq = list(task_return)
                        elif len(online_tasks) >= 1 and len(outer_dict[self.car2set[agv + 1]]) >= 1:  # 指派一个任务
                            assign_task = online_tasks.pop(0)  # 待分配的任务id
                            end = (outer_dict[self.car2set[agv + 1]][0])
                            task_dict[assign_task].end = end
                            task_get = self.get_route(start=self.park_list[agv],
                                                      end=task_dict[assign_task].start)
                            task_put = self.get_route(start=task_dict[assign_task].start,
                                                      end=task_dict[assign_task].end)
                            task_return = self.get_route(start=task_dict[assign_task].end, end=self.park_list[agv])
                            # new task list
                            new_task_dict[agv + 1] = [-int(agv + 1), assign_task, -int(agv + 1)]
                            # change task_dict
                            task_dict[assign_task].route_seq = list(task_get) + list(task_put[1:])
                            task_dict[-int(agv+1)].route_seq = list(task_return)
                        else:
                            new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out
                    else:  # 当前有任务在执行
                        assert agv_status_list[agv] in ('get', 'put')  # [1, 2, -agv]
                        exec_task = agv_task_list[agv][0]
                        # 只能指派一个任务
                        if len(online_tasks) >= 1 and len(outer_dict[self.car2set[agv + 1]]) >= 1:  # 指派一个任务
                            assign_task = online_tasks.pop(0)  # 待分配的任务id
                            end = (outer_dict[self.car2set[agv + 1]][0])
                            task_dict[assign_task].end = end
                            task_get = self.get_route(start=task_dict[exec_task].end,
                                                      end=task_dict[assign_task].start)
                            task_put = self.get_route(start=task_dict[assign_task].start,
                                                      end=task_dict[assign_task].end)
                            task_return = self.get_route(start=task_dict[assign_task].end, end=self.park_list[agv])
                            # new task list
                            new_task_dict[agv + 1] = [exec_task, assign_task, -int(agv + 1)]
                            # change task_dict
                            task_dict[assign_task].route_seq = list(task_get) + list(task_put[1:])
                            task_dict[-int(agv+1)].route_seq = list(task_return)
                        else:
                            new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out
                else:
                    # [-agv, 1, 2, -agv]
                    assert len(agv_task_list[agv]) == 4
                    assert agv_status_list[agv] == 'return' and agv_task_list[agv][0] == -(agv + 1), \
                        f'{agv_status_list[agv], agv_task_list[agv][0]}'
                    # 指派两个任务，取3保证一定宽放量，避免有车空闲却无任务可接
                    if len(online_tasks) >= 3 and len(outer_dict[self.car2set[agv + 1]]) >= 2:
                        assign_task_1 = online_tasks.pop(0)  # 1st待分配的任务id
                        assign_task_2 = online_tasks.pop(0)  # 2nd待分配的任务id
                        end_1, end_2 = (outer_dict[self.car2set[agv + 1]][i] for i in range(2))
                        task_dict[assign_task_1].end, task_dict[assign_task_2].end = end_1, end_2
                        task_1_get = self.get_route(start=self.park_list[agv],
                                                    end=task_dict[assign_task_1].start)
                        task_1_put = self.get_route(start=task_dict[assign_task_1].start,
                                                    end=task_dict[assign_task_1].end)
                        task_2_get = self.get_route(start=task_dict[assign_task_1].end,
                                                    end=task_dict[assign_task_2].start)
                        task_2_put = self.get_route(start=task_dict[assign_task_2].start,
                                                    end=task_dict[assign_task_2].end)
                        task_return = self.get_route(start=task_dict[assign_task_2].end,
                                                     end=self.park_list[agv])
                        # new task list
                        new_task_dict[agv + 1] = [-int(agv + 1), assign_task_1, assign_task_2, -int(agv + 1)]
                        # change task_dict
                        task_dict[assign_task_1].route_seq = list(task_1_get) + list(task_1_put[1:])
                        task_dict[assign_task_2].route_seq = list(task_2_get) + list(task_2_put[1:])
                        task_dict[-int(agv+1)].route_seq = list(task_return)
                    elif len(online_tasks) >= 1 and len(outer_dict[self.car2set[agv + 1]]) >= 1:  # 指派一个任务
                        assign_task = online_tasks.pop(0)  # 待分配的任务id
                        end = (outer_dict[self.car2set[agv + 1]][0])
                        task_dict[assign_task].end = end
                        task_get = self.get_route(start=self.park_list[agv],
                                                  end=task_dict[assign_task].start)
                        task_put = self.get_route(start=task_dict[assign_task].start,
                                                  end=task_dict[assign_task].end)
                        task_return = self.get_route(start=task_dict[assign_task].end, end=self.park_list[agv])
                        # new task list
                        new_task_dict[agv + 1] = [-int(agv + 1), assign_task, -int(agv + 1)]
                        # change task_dict
                        task_dict[assign_task].route_seq = list(task_get) + list(task_put[1:])
                        task_dict[-int(agv+1)].route_seq = list(task_return)
                    else:
                        new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out
            else:  # 离线任务
                if len(agv_task_list[agv]) == 0 or len(agv_task_list[agv]) == 1:  # 没有当前任务或只有一个任务在执行（return）
                    if len(agv_task_list[agv]) == 0:
                        assert agv_loc[agv] == self.park_list[
                            agv], f'agv{agv + 1}在任务列表长度为0时位置错误，cur_loc={agv_loc[agv]}'
                    else:
                        assert agv_task_list[agv][0] == -(agv + 1) and agv_status_list[
                            agv] == 'return', f'agv{agv}, task_list_len = 1'
                    if agv2offlinetask[agv + 1]:  # 后续有任务
                        if len(agv2offlinetask[agv + 1]) >= 2:  # 分配两个任务
                            assign_task_1 = agv2offlinetask[agv + 1].pop(0)  # 1st待分配的任务id
                            assign_task_2 = agv2offlinetask[agv + 1].pop(0)  # 2nd待分配的任务id
                            task_1_get = self.get_route(start=self.park_list[agv],
                                                        end=task_dict[assign_task_1].start)
                            task_1_put = self.get_route(start=task_dict[assign_task_1].start,
                                                        end=task_dict[assign_task_1].end)
                            task_2_get = self.get_route(start=task_dict[assign_task_1].end,
                                                        end=task_dict[assign_task_2].start)
                            task_2_put = self.get_route(start=task_dict[assign_task_2].start,
                                                        end=task_dict[assign_task_2].end)
                            task_return = self.get_route(start=task_dict[assign_task_2].end,
                                                         end=self.park_list[agv])
                            # new task list
                            if len(agv_task_list[agv]) == 0:
                                new_task_dict[agv + 1] = [assign_task_1, assign_task_2, -int(agv + 1)]
                            else:
                                new_task_dict[agv + 1] = [-int(agv + 1), assign_task_1, assign_task_2,
                                                          -int(agv + 1)]
                            # change task_dict
                            task_dict[assign_task_1].route_seq = list(task_1_get) + list(task_1_put[1:])
                            task_dict[assign_task_2].route_seq = list(task_2_get) + list(task_2_put[1:])
                            task_dict[-int(agv+1)].route_seq = list(task_return)
                        elif len(agv2offlinetask[agv + 1]) >= 1:  # 分配一个任务
                            assign_task = agv2offlinetask[agv + 1].pop(0)  # 待分配的任务id
                            task_get = self.get_route(start=self.park_list[agv],
                                                      end=task_dict[assign_task].start)
                            task_put = self.get_route(start=task_dict[assign_task].start,
                                                      end=task_dict[assign_task].end)
                            task_return = self.get_route(start=task_dict[assign_task].end, end=self.park_list[agv])
                            # new task list
                            if len(agv_task_list[agv]) == 0:
                                new_task_dict[agv + 1] = [assign_task, -int(agv + 1)]
                            else:
                                new_task_dict[agv + 1] = [-int(agv + 1), assign_task, -int(agv + 1)]
                            # change task_dict
                            task_dict[assign_task].route_seq = list(task_get) + list(task_put[1:])
                            task_dict[-int(agv+1)].route_seq = list(task_return)
                    else:  # 没有后续任务
                        new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out
                elif len(agv_task_list[agv]) == 2:  # 只有一个当前任务在执行
                    assert agv_status_list[agv] in ('get', 'put')
                    if len(agv2offlinetask[agv + 1]) >= 1:  # 指派一个任务
                        exec_task = agv_task_list[agv][0]
                        assign_task = agv2offlinetask[agv + 1].pop(0)  # 待分配的任务id
                        task_get = self.get_route(start=task_dict[exec_task].end,
                                                  end=task_dict[assign_task].start)
                        task_put = self.get_route(start=task_dict[assign_task].start,
                                                  end=task_dict[assign_task].end)
                        task_return = self.get_route(start=task_dict[assign_task].end, end=self.park_list[agv])
                        # new task list
                        new_task_dict[agv + 1] = [exec_task, assign_task, -int(agv + 1)]
                        # change task_dict
                        task_dict[assign_task].route_seq = list(task_get) + list(task_put[1:])
                        task_dict[-int(agv+1)].route_seq = list(task_return)
                    else:  # 不需要指派任务
                        new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out
                elif len(agv_task_list[agv]) == 3:  # 当前三个任务在列表中
                    if agv_status_list[agv] == 'return':  # [-agv, 1, -agv]
                        # 指派两个任务，取3保证一定宽放量，避免有车空闲却无任务可接
                        if len(agv2offlinetask[agv + 1]) >= 2:
                            assign_task_1 = agv2offlinetask[agv + 1].pop(0)  # 1st待分配的任务id
                            assign_task_2 = agv2offlinetask[agv + 1].pop(0)  # 2nd待分配的任务id
                            task_1_get = self.get_route(start=self.park_list[agv],
                                                        end=task_dict[assign_task_1].start)
                            task_1_put = self.get_route(start=task_dict[assign_task_1].start,
                                                        end=task_dict[assign_task_1].end)
                            task_2_get = self.get_route(start=task_dict[assign_task_1].end,
                                                        end=task_dict[assign_task_2].start)
                            task_2_put = self.get_route(start=task_dict[assign_task_2].start,
                                                        end=task_dict[assign_task_2].end)
                            task_return = self.get_route(start=task_dict[assign_task_2].end,
                                                         end=self.park_list[agv])
                            # new task list
                            new_task_dict[agv + 1] = [-int(agv + 1), assign_task_1, assign_task_2, -int(agv + 1)]
                            # change task_dict
                            task_dict[assign_task_1].route_seq = list(task_1_get) + list(task_1_put[1:])
                            task_dict[assign_task_2].route_seq = list(task_2_get) + list(task_2_put[1:])
                            task_dict[-int(agv+1)].route_seq = list(task_return)
                        elif len(agv2offlinetask[agv + 1]) >= 1:  # 指派一个任务
                            assign_task = agv2offlinetask[agv + 1].pop(0)  # 待分配的任务id
                            task_get = self.get_route(start=self.park_list[agv],
                                                      end=task_dict[assign_task].start)
                            task_put = self.get_route(start=task_dict[assign_task].start,
                                                      end=task_dict[assign_task].end)
                            task_return = self.get_route(start=task_dict[assign_task].end, end=self.park_list[agv])
                            # new task list
                            new_task_dict[agv + 1] = [-int(agv + 1), assign_task, -int(agv + 1)]
                            # change task_dict
                            task_dict[assign_task].route_seq = list(task_get) + list(task_put[1:])
                            task_dict[-int(agv+1)].route_seq = list(task_return)
                        else:  # 没有任务可以指派
                            new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out
                    else:  # 当前有任务在执行
                        assert agv_status_list[agv] in ('get', 'put')  # [1, 2, -agv]
                        exec_task = agv_task_list[agv][0]
                        # 只能指派一个任务
                        if len(agv2offlinetask[agv + 1]) >= 1:  # 指派一个任务
                            assign_task = agv2offlinetask[agv + 1].pop(0)  # 待分配的任务id
                            task_get = self.get_route(start=task_dict[exec_task].end,
                                                      end=task_dict[assign_task].start)
                            task_put = self.get_route(start=task_dict[assign_task].start,
                                                      end=task_dict[assign_task].end)
                            task_return = self.get_route(start=task_dict[assign_task].end, end=self.park_list[agv])
                            # new task list
                            new_task_dict[agv + 1] = [exec_task, assign_task, -int(agv + 1)]
                            # change task_dict
                            task_dict[assign_task].route_seq = list(task_get) + list(task_put[1:])
                            task_dict[-int(agv+1)].route_seq = list(task_return)
                        else:
                            new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out
                else:  # 当前列表中有四个任务, [-agv, 1, 2, -agv]
                    assert len(agv_task_list[agv]) == 4
                    assert agv_status_list[agv] == 'return' and agv_task_list[agv][0] == -(agv + 1), \
                        f'{agv_status_list[agv], agv_task_list[agv][0]}'
                    # 指派两个任务，取3保证一定宽放量，避免有车空闲却无任务可接
                    if len(agv2offlinetask[agv + 1]) >= 2:
                        assign_task_1 = agv2offlinetask[agv + 1].pop(0)  # 1st待分配的任务id
                        assign_task_2 = agv2offlinetask[agv + 1].pop(0)  # 2nd待分配的任务id
                        task_1_get = self.get_route(start=self.park_list[agv],
                                                    end=task_dict[assign_task_1].start)
                        task_1_put = self.get_route(start=task_dict[assign_task_1].start,
                                                    end=task_dict[assign_task_1].end)
                        task_2_get = self.get_route(start=task_dict[assign_task_1].end,
                                                    end=task_dict[assign_task_2].start)
                        task_2_put = self.get_route(start=task_dict[assign_task_2].start,
                                                    end=task_dict[assign_task_2].end)
                        task_return = self.get_route(start=task_dict[assign_task_2].end,
                                                     end=self.park_list[agv])
                        # new task list
                        new_task_dict[agv + 1] = [-int(agv + 1), assign_task_1, assign_task_2, -int(agv + 1)]
                        # change task_dict
                        task_dict[assign_task_1].route_seq = list(task_1_get) + list(task_1_put[1:])
                        task_dict[assign_task_2].route_seq = list(task_2_get) + list(task_2_put[1:])
                        task_dict[-int(agv+1)].route_seq = list(task_return)
                    elif len(agv2offlinetask[agv + 1]) >= 1:  # 指派一个任务
                        assign_task = agv2offlinetask[agv + 1].pop(0)  # 待分配的任务id
                        task_get = self.get_route(start=self.park_list[agv],
                                                  end=task_dict[assign_task].start)
                        task_put = self.get_route(start=task_dict[assign_task].start,
                                                  end=task_dict[assign_task].end)
                        task_return = self.get_route(start=task_dict[assign_task].end, end=self.park_list[agv])
                        # new task list
                        new_task_dict[agv + 1] = [-int(agv + 1), assign_task, -int(agv + 1)]
                        # change task_dict
                        task_dict[assign_task].route_seq = list(task_get) + list(task_put[1:])
                        task_dict[-int(agv+1)].route_seq = list(task_return)
                    else:
                        new_task_dict[agv + 1] = agv_task_list[agv + 1]  # todo: check this out

        assert None not in new_task_dict.values()
        return new_task_dict
