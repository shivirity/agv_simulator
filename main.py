from Prepare import Problem
from initialize import dictionary_task, dictionary_car, dictionary_task_online_ending_order
from Map import dictionary_map

from agv_control.RouteControl import RouteController

if __name__ == '__main__':

    # 数据读入

    problem = Problem(dictionary_map, dictionary_task, dictionary_car, 'uniform', 216, 50)

    first_in = True

    while not problem.is_finished:

        # 在线任务更新
        problem.renew_task_online(problem.Task)  # 更新在线任务的到达
        # 系统静态更新
        if problem.time % 216 == 0 or problem.online_task_arrival:  # 路径规划的更新节点判断条件
            task_dict, route_seq = problem.route_scheduling()

            problem.update_car(task_dict=task_dict)

            # 初始化controller
            if first_in:
                problem.controller = RouteController(
                    routes={i: problem.AGV[i+1].route[2:] for i in range(8)}
                )
                first_in = False
            else:
                problem.controller.update_routes(routes={i: problem.AGV[i+1].route[2:] for i in range(8)})

        # 系统动态更新
        problem.run_step()
        # problem.time += problem.step
