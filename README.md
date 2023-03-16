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
```python
time span = 12.34s  # 仿真运行时间
operation time = 22902s  # 仿真内部仓储过程时间
task end time = [12582, 13856, 18512, 21492, 18802, 22902, 21382, 10728] in seconds  # 各车执行完各自任务的时间
deadlock times = 2  # 死锁发生次数
loc collision times = 0  # 路径节点潜在冲突次数
side collision times = 0  # 库位节点潜在冲突次数
```