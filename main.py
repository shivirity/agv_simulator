import time
import logging
import colorlog

from Prepare import Problem
from initialize import dictionary_task, dictionary_car, dictionary_task_online_ending_order
from Map import dictionary_map

from agv_control.RouteControl import RouteController
from RoutePlan_basic import Routeplanner_basic

# set logger
log_colors_config = {
    'DEBUG': 'cyan',
    'INFO': 'green',
    'WARNING': 'yellow',
    'ERROR': 'red',
    'CRITICAL': 'bold_red',
}
logger = logging.getLogger("logger_main")
logger.setLevel(logging.DEBUG)
sh = logging.StreamHandler()
sh.setLevel(logging.INFO)
stream_fmt = colorlog.ColoredFormatter(
    fmt="%(log_color)s[%(asctime)s] - %(filename)-8s - %(levelname)-7s - line %(lineno)s - %(message)s",
    log_colors=log_colors_config)
sh.setFormatter(stream_fmt)
logger.addHandler(sh)
sh.close()

log_print_freq = 3000  # (set None to avoid printing log)

planner = Routeplanner_basic
controller = RouteController

if __name__ == '__main__':

    # 数据读入

    problem = Problem(dictionary_map, dictionary_task, dictionary_car,
                      dictionary_task_online_ending_order, 'uniform', 216, 50)

    first_in = True
    route_planner = planner(grids=problem.Map)

    start = time.time()

    while not problem.is_finished:

        # 在线任务更新
        problem.renew_task_online(problem.Task)  # 更新在线任务的到达
        # 系统静态更新

        if problem.time % 216 == 0 or problem.online_task_arrival:  # 路径规划的更新节点判断条件

            task_dict = route_planner.route_scheduling(
                agv_loc=problem.get_agv_location(),
                agv_task_list=problem.get_agv_tasklist(),
                agv_status_list=problem.get_agv_task_status(),
                task_to_plan=problem.get_task_not_perform(),
                task_dict=problem.Task,
                outer_dict=problem.Task_order
            )
            problem.update_car(task_dict=task_dict)

            # 初始化controller
            if first_in:
                problem.controller = controller(
                    routes={i: problem.AGV[i+1].route[2:] for i in range(8)}
                )
                first_in = False
            else:
                problem.controller.update_routes(routes={i: problem.AGV[i+1].route[2:] for i in range(8)})

        # 系统动态更新
        problem.run_step()
        if problem.time % int(log_print_freq) == 0 and log_print_freq is not None:
            logger.info(f'time={problem.time}, cur_agv_loc={problem.get_agv_location()}')

    else:
        end = time.time()
        logger.info('time span = {:.2f}s'.format(end-start))
        logger.info(f'operation time = {problem.time}s')
        logger.info(
            f'task end time = '
            f'{[(problem.AGV[i].end_state[0] if problem.AGV[i].end_state[0] > 0 else problem.time) for i in range(1, 9)]} in seconds')
        logger.info(
            f'agv wait time = '
            f'{[problem.AGV[i].waiting_time for i in range(1, 9)]} in seconds')
        logger.info(
            f'avg agv wait time = '
            f'{int(sum([problem.AGV[i].waiting_time for i in range(1, 9)])/8)} in seconds')
        logger.info(f'deadlock times = {problem.deadlock_times}')
        logger.info(f'loc collision times = {problem.loc_error}')
        logger.info(f'side collision times = {problem.side_error}')
