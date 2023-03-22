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
planner = Routeplanner_basic  # 路径规划控制器类选择
controller = RouteController  # 控制策略控制器类选择
```
## Results
### baseline
```python
time span = 36.88s  # 仿真运行时间
operation time = 22744s  # 仿真内部仓储过程时间
task end time = [16786, 17158, 17792, 18546, 22706, 22712, 22710, 22744] in seconds  # 各车执行完各自任务的时间
agv wait time = [11542, 11622, 10722, 10388, 13208, 12062, 13550, 13242] in seconds  # 各车因控制策略而等待的时间
avg agv wait time = 12042 in seconds  # 各车平均等待时间
deadlock times = 0  # 死锁发生次数
loc collision times = 0  # 路径节点潜在冲突次数
side collision times = 0  # 库位节点潜在冲突次数
```
### our method
```python
param setting: online_first_rate = 0.6
time span = 369.36s
operation time = 21152s
task end time = [11342, 13394, 15548, 10586, 21152, 20312, 19102, 12394] in seconds
agv wait time = [5372, 7440, 7748, 980, 9444, 10712, 10780, 3320] in seconds
avg agv wait time = 6974 in seconds
deadlock times = 1
loc collision times = 0
side collision times = 42
```
