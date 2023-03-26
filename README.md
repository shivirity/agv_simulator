# agv_simulator
simulator for agv scheduling system (Project 2023, SJTU)

Requirements are needed for log printing and sci-computing, please use this command in your env or venv.
```python
pip install -r requirements.txt
```

## Run file
run ```main.py``` with parameter settings:
```python
log_print_freq = 3000  # 日志显示间隔时间
planner_choose = 'new'  # 路径规划控制器类选择, options: ['baseline', 'new']
controller_choose = RouteController  # 控制策略控制器类选择, options: [RouteController, RouteController_basic, RouteController_basic_multitask]
```
choose different dataset with parameter setting in ```Prepare.py```
```python
dataset_choose = 200  # 不同任务数量或类型
# options: [80, 120, 160, 200, 8080(online), 2333(offline)]
```
## Results
### number of tasks = 200
#### baseline
```python
operation time = 21694s  # 仿真内部仓储过程时间
task end time = [16360, 16732, 17682, 18554, 21654, 21624, 21666, 21694] in seconds  # 各车执行完各自任务的时间
agv wait time = [11050, 11332, 9844, 10242, 11986, 12156, 11520, 12408] in seconds  # 各车因控制策略而等待的时间
avg agv wait time = 11317 in seconds  # 各车因控制策略而等待的平均时间
deadlock times = 0  # 死锁发生次数
loc collision times = 15  # 路径节点潜在冲突次数
side collision times = 0  # 库位节点潜在冲突次数
```
#### our method
```python
operation time = 15584s
task end time = [8376, 9568, 13588, 7888, 14664, 15188, 15584, 11384] in seconds
agv wait time = [1182, 1610, 1174, 166, 5700, 5558, 6568, 2106] in seconds
avg agv wait time = 3008 in seconds
deadlock times = 0
loc collision times = 1
side collision times = 6
```

### number of tasks = 160
#### baseline
```python
operation time = 16632s
task end time = [12506, 12952, 13744, 14316, 16622, 16600, 16602, 16632] in seconds
agv wait time = [8202, 8552, 7636, 7414, 9066, 8270, 9244, 9222] in seconds
avg agv wait time = 8450 in seconds
deadlock times = 0
loc collision times = 10
side collision times = 0
```
#### our method
```python
operation time = 13224s
task end time = [8518, 8794, 11704, 6490, 13224, 12934, 12586, 9292] in seconds
agv wait time = [3298, 3792, 4686, 168, 5690, 5512, 5132, 1830] in seconds
avg agv wait time = 3763 in seconds
deadlock times = 0
loc collision times = 0
side collision times = 7
```

### number of tasks = 120
#### baseline
```python
operation time = 12532s
task end time = [8852, 9422, 10358, 10916, 12518, 12532, 12528, 12504] in seconds
agv wait time = [5500, 5934, 5734, 4906, 6530, 6260, 7000, 6784] in seconds
avg agv wait time = 6081 in seconds
deadlock times = 0
loc collision times = 9
side collision times = 0
```
#### our method
```python
operation time = 9662s
task end time = [5710, 5884, 7740, 5220, 9662, 9294, 9390, 6662] in seconds
agv wait time = [988, 1208, 698, 222, 3902, 3694, 3836, 950] in seconds
avg agv wait time = 1937 in seconds
deadlock times = 0
loc collision times = 1
side collision times = 4
```

### number of tasks = 80
#### baseline
```python
operation time = 7792s
task end time = [5324, 5686, 6542, 7254, 7792, 7768, 7784, 7766] in seconds
agv wait time = [2980, 3346, 2942, 2354, 3994, 3444, 4092, 3950] in seconds
avg agv wait time = 3387 in seconds
deadlock times = 0
loc collision times = 7
side collision times = 0
```
#### our method
```python
operation time = 6992s
task end time = [3496, 4340, 4896, 3996, 6604, 6968, 6992, 4858] in seconds
agv wait time = [720, 900, 676, 374, 2758, 3084, 3232, 1052] in seconds
avg agv wait time = 1599 in seconds
deadlock times = 0
loc collision times = 0
side collision times = 0
```
