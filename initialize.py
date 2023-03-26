import pandas as pd
from Prepare import AGV, Task, dataset_choose
from numpy import random
import numpy as np

# 统一读入配置文件
df_task_offline = pd.read_csv(f"config/task_{dataset_choose}/OfflineTask.csv")
df_task_online = pd.read_csv(f"config/task_{dataset_choose}/OnlineTask.csv")
df_task_online_order = pd.read_csv(f"config/task_{dataset_choose}/OnlineTaskOrder.csv")

# 标记叉车初始位置
df_loc = pd.read_csv(f"config/task_{dataset_choose}/Car.csv")
init_loc = df_loc['park'].tolist()
# init_loc = [364, 362, 360, 354, 66, 57, 135, 140]  # 叉车初始坐标


def sp_exponential(t, size):
    r = random.exponential(t, size)
    return np.cumsum(r)


# 创建任务字典
# 离线任务(在线要写到Problem类里)
def create_task_offline(df_task_off):
    dict_task = {}
    for i in range(df_task_off.shape[0]):
        task_tmp = Task(df_task_off.iloc[i, 0], 0, 0, df_task_off.iloc[i, 1],
                        df_task_off.iloc[i, 2], 1, df_task_off.iloc[i, 3])
        # todo 如果离线任务需要修改指派，此处的任务状态需要修改
        dict_task[i+1] = task_tmp
    for i in range(1, 9):  # 此处没有用叉车的数量，直接用了8
        dict_task[-i] = Task(
            num=-i, arrival=None, task_type=None, start=init_loc[i-1], end=init_loc[i-1], state=None, car=i)  # 更新每辆车的return任务
    return dict_task


def create_task_online_order(df_task_on_order, df_car):
    dict_end_online = {}
    for i in range(df_task_on_order.shape[1] - 1):
        # 输出车辆编号对应的区域的终点选择优先级序列
        dict_end_online[i+1] = df_task_on_order[df_car[df_car['id'] == i+1].iloc[0, 2]].tolist()
    return dict_end_online


# 创建无人叉车单位：
def create_agvs(init_location):
    dict_agv = {}
    for i in range(len(init_location)):
        agv_tmp = AGV(i+1, init_location[i], init_location[i], [])
        dict_agv[i+1] = agv_tmp
    return dict_agv


dictionary_task = create_task_offline(df_task_offline)
dictionary_task_online_ending_order = create_task_online_order(df_task_online_order, df_loc)
dictionary_car = create_agvs(init_loc)
