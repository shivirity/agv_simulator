import pandas as pd

# 统一读入配置文件
df_Grid = pd.read_csv("config/Map.csv")
df_Inventory = pd.read_csv("config/Inventory.csv")


# 节点类的形式储存地图数据
class Grid:
    def __init__(self, grid_id, x, y, grid_type, neighbor, state, conflict_list):
        self.id = grid_id  # 地图节点的编号
        self.x = x  # 节点横坐标
        self.y = y  # 节点纵坐标
        self.type = grid_type  # 节点的类型 道路/停靠点/货位
        self.neighbor = neighbor
        self.state = state  # 节点的库存配置状态，-1代表道路节点，0 代表无货，1 代表有货
        self.conflict = conflict_list  # 每个节点的冲突节点列表，-1为路径节点，列表为库位节点


# 初始化函数读入配置文件
def create_map(df_grid, df_inventory):
    dict_map = {}
    for i in range(df_grid.shape[0]):
        neighbour = df_grid.iloc[i, 4].split(',')
        neighbour = [int(x) for x in neighbour]
        conflict = df_grid.iloc[i, 5].split(',')
        conflict = [int(x) for x in conflict]
        grid_tmp = Grid(df_grid.iloc[i, 0], df_grid.iloc[i, 1], df_grid.iloc[i, 2], df_grid.iloc[i, 3],
                        neighbour, df_inventory.iloc[i, 1], conflict)
        dict_map[i+1] = grid_tmp
    return dict_map


dictionary_map = create_map(df_Grid, df_Inventory)
