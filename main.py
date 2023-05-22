import time
import logging
import colorlog

from Prepare import dataset_choose, Problem
from initialize import dictionary_task, dictionary_car, dictionary_task_online_ending_order
from Map import dictionary_map

# route_plan
from RoutePlan_basic import Routeplanner_basic
# route_control
from agv_control.RouteControl_basic_multitask import RouteController_basic_multitask
from agv_control.RouteControl_basic import RouteController_basic
from agv_control.RouteControl_basic_cor import RouteController_basic_cor
from agv_control.RouteControl import RouteController

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
planner_choose = 'baseline'  # 'new' or 'baseline'
controller_choose = RouteController

# new route_scheduling parameters
candidate_path_num = 5
max_agv_task_num = 2

# size parameter
size_dict = {i: [int(i/2), int(i/2)] for i in [80, 120, 160, 200]}
size_dict[8080], size_dict[2333] = [0, 100], [100, 0]

time_list = []
collision_list = []
deadlock_list = []
wait_list = []
loc_list = []
task_list = []
process_list = []

if __name__ == '__main__':

    # 数据读入
    problem = Problem(dictionary_map, dictionary_task, dictionary_car,
                      dictionary_task_online_ending_order, 'uniform', 216, size_dict.get(dataset_choose, -1))

    first_in = True
    route_planner = Routeplanner_basic(grids=problem.Map, size=size_dict[dataset_choose])

    start = time.time()

    while not problem.is_finished:

        # 在线任务更新
        problem.renew_task_online(problem.Task)  # 更新在线任务的到达
        # 系统静态更新

        if problem.time % 216 == 0 or problem.online_task_arrival:  # 路径规划的更新节点判断条件

            # 设定候选路径集数量及每次更新后叉车上最多的任务数
            if planner_choose == 'new':
                task_dict = problem.route_scheduling(
                    time=problem.time, agv_max_task_num=max_agv_task_num, candidate_num=candidate_path_num)
            elif planner_choose == 'baseline':
                task_dict = route_planner.route_scheduling(
                    agv_loc=problem.get_agv_location(),
                    agv_task_list=problem.get_agv_tasklist(),
                    agv_status_list=problem.get_agv_task_status(),
                    task_to_plan=problem.get_task_not_perform(),
                    task_dict=problem.Task,
                    outer_dict=problem.Task_order
                )
            else:
                logger.warning(f'planner choose wrong!')
                assert False

            problem.update_car(task_dict=task_dict)

            # 初始化controller
            if first_in:
                problem.controller = controller_choose(
                    routes={i: problem.AGV[i+1].route[2:] for i in range(8)},
                    size=size_dict[dataset_choose]
                )
                first_in = False
            else:
                problem.controller.update_routes(routes={i: problem.AGV[i+1].route[1:] for i in range(8)})

        # 系统动态更新
        problem.run_step()
        if problem.time % int(log_print_freq) == 0 and log_print_freq is not None:
            logger.info(f'time={problem.time}, cur_agv_loc={problem.get_agv_location()}')
        # logger.info(f'0: {problem.controller.shared_routes[0]}')
        # logger.info(f'2: {problem.controller.shared_routes[2]}')
        if problem.time == 2810:
            logger.debug('')
        # information collection
        time_list.append(problem.time)
        collision_list.append(problem.loc_error_count + problem.side_error_count)
        deadlock_list.append(problem.deadlock_times)
        wait_list.append([problem.AGV[i].waiting_time_work+problem.AGV[i].waiting_time_park for i in range(1, 9)])
        loc_list.append([problem.AGV[i].location for i in range(1, 9)])
        task_list.append([problem.AGV[i].task for i in range(1, 9)])
        process_list.append([problem.AGV[i].get_process(problem.Task) for i in range(1, 9)])

    else:
        end = time.time()
        logger.info('time span = {:.2f}s'.format(end-start))
        logger.info(f'operation time = {problem.time}s')
        logger.info(
            f'task end time = '
            f'{[(problem.AGV[i].end_state[0] if problem.AGV[i].end_state[0] > 0 else problem.time) for i in range(1, 9)]} in seconds')
        logger.info(
            f'agv wait time(work) = '
            f'{[problem.AGV[i].waiting_time_work for i in range(1, 9)]} in seconds')
        logger.info(
            f'agv wait time(park) = '
            f'{[problem.AGV[i].waiting_time_park for i in range(1, 9)]} in seconds')
        logger.info(
            f'avg agv wait time = '
            f'{int(sum([problem.AGV[i].waiting_time for i in range(1, 9)])/8)} in seconds')
        logger.info(
            f'avg route_seq length = '
            f'{sum([len(problem.Task[i].route_seq) for i in problem.Task.keys() if i > 0])/(len(problem.Task.keys())-8)}'
        )
        logger.info(f'deadlock times = {problem.deadlock_times}')
        logger.info(f'loc collision times = {problem.loc_error_count}')
        logger.info(f'side collision times = {problem.side_error_count}')

        info_dict = {
            'time': time_list,
            'collision': collision_list,
            'deadlock': deadlock_list,
            'wait': wait_list,
            'loc': loc_list,
            'task': task_list,
            'process': process_list
        }
        import pickle
        with open("info.pkl", 'wb') as f:
            pickle.dump(info_dict, f)
