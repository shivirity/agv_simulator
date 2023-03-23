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
controller = RouteController  # 控制策略控制器类选择, options: [RouteController, RouteController_basic, RouteController_basic_multitask]
```
## Results
### baseline
```python
time span = 17.56s  # 仿真运行时间
operation time = 21186s  # 仿真内部仓储过程时间
task end time = [16066, 15696, 17442, 18046, 21140, 21138, 21140, 21186] in seconds  # 各车执行完各自任务的时间
agv wait time = [10834, 10040, 7878, 9434, 11376, 10722, 12152, 11722] in seconds  # 各车因控制策略而等待的时间
avg agv wait time = 10519 in seconds  # 各车平均等待时间
deadlock times = 0  # 死锁发生次数
loc collision times = 2  # 路径节点潜在冲突次数
side collision times = 0  # 库位节点潜在冲突次数
```
### our method
```python
time span = 136.09s
operation time = 15584s
task end time = [10586, 10586, 13588, 10586, 14664, 15188, 15584, 11384] in seconds
agv wait time = [1182, 1610, 1174, 166, 5700, 5558, 6568, 2106] in seconds
avg agv wait time = 3008 in seconds
deadlock times = 0
loc collision times = 1
side collision times = 37
```

