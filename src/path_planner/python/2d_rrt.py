# import matplotlib.pyplot as plt
# import custom_tree
# from matplotlib.patches import Circle
# import numpy as np

# # 构建圆形障碍物：x,y,radius
# circles = [[30,20,10],[40,45,8]]
# step = 5 #步长
# max_iter = 5000 #最大迭代次数
# pos_init = [0,0]
# pos_goal = [90,60]
# threshold_dis = 5

# def generate_random():
#     # 10%概率直接采样目标点，加速收敛
#     if np.random.random() < 0.1:
#         return pos_goal.copy()
#     else:
#         # 在整个空间内随机采样
#         return [np.random.uniform(0, 90), np.random.uniform(0, 60)]

# def line_segment_circle_collision(p1, p2, circle_center, circle_radius):
#     # 线段的方向向量
#     line_vec = [p2[0] - p1[0], p2[1] - p1[1]]
    
#     # 线段起点到圆心的向量
#     to_center = [circle_center[0] - p1[0], circle_center[1] - p1[1]]
#     # 线段长度的平方
#     line_len_sq = line_vec[0]**2 + line_vec[1]**2
#     # 计算最近点参数 t
#     t = (to_center[0] * line_vec[0] + to_center[1] * line_vec[1]) / line_len_sq if line_len_sq != 0 else 0
#     # 限制 t 在 [0, 1] 范围内，表示线段上的点
#     t = max(0, min(1, t))
#     # 计算线段上距离圆心最近的点
#     closest_point = [p1[0] + t * line_vec[0], p1[1] + t * line_vec[1]]
#     # 计算最近点到圆心的距离
#     dist_sq = (closest_point[0] - circle_center[0])**2 + (closest_point[1] - circle_center[1])**2
#     # 判断是否碰撞
#     return dist_sq <= circle_radius**2



# def free_collision(random,tree):
#     # 先找出离随机点距离最近的节点
#     min_distance = pow(random[0]-tree.nodes[0].pos[0],2)+pow(random[1]-tree.nodes[0].pos[1],2)
#     nearest_node = tree.nodes[0]
#     id = 0
#     for i in range(len(tree.nodes)):
#         distance = pow(random[0]-tree.nodes[i].pos[0],2)+pow(random[1]-tree.nodes[i].pos[1],2)
#         if distance < min_distance:
#             nearest_node = tree.nodes[i]
#             id = i
    
#     # 计算新节点位置（沿最近节点到随机点方向扩展step距离）
#     dx = random[0] - nearest_node.pos[0]
#     dy = random[1] - nearest_node.pos[1]
#     dist = np.sqrt(dx**2 + dy**2)
    
#     if dist == 0:  # 避免除以零
#         return False, None, -1
    
#     # 归一化方向向量
#     dir_x = dx / dist
#     dir_y = dy / dist
    
#     # 生成新节点位置
#     new_pos = [
#         nearest_node.pos[0] + step * dir_x,
#         nearest_node.pos[1] + step * dir_y
#     ]
    
#     # 判断随机点与该节点构成的线段有没有与障碍物碰撞
#     flag = True
#     for circle in circles:
#         center = [circle[0],circle[1]]
#         radius = circle[2]
#         if line_segment_circle_collision(random,nearest_node.pos,center,radius):
#             flag = False
#             break;
    
#     new_node = custom_tree.Node(pos=new_pos, parent_id=id)
#     return flag,new_node,id


# root = custom_tree.Node(pos=pos_init)
# tree = custom_tree.Tree()
# tree.add_node(root)

# find_path = False

# # 创建绘图对象
# fig, ax = plt.subplots()

# # 添加圆形 patch
# for i in range(len(circles)):
#     circle = Circle((circles[i][0], circles[i][1]), circles[i][2], fill=True, color='blue') 
#     ax.add_artist(circle)

# for i in range(max_iter):
#     random = generate_random()
#     flag,new_node,id= free_collision(random=random,tree=tree)
#     if not flag:
#         continue;
#     tree.add_node(new_node)
#         # 获取父节点位置
#     parent_pos = tree.nodes[new_node.parent_id].pos
    
#     # 绘制从父节点到新节点的线段
#     ax.plot(
#         [parent_pos[0], new_node.pos[0]],  # x坐标列表
#         [parent_pos[1], new_node.pos[1]],  # y坐标列表
#         'k-',  # 黑色实线
#         alpha=0.5,  # 半透明
#         linewidth=1  # 线宽
#     )
#     # 新生成节点与目标点距离
#     dis = np.sqrt(pow(pos_goal[0]-new_node.pos[0],2)+pow(pos_goal[1]-new_node.pos[1],2))
#     if(dis < threshold_dis):
#         find_path = True
#         break;

# # 设置坐标轴范围（可根据实际情况调整，确保圆完整显示）
# ax.set_xlim(0, 90)
# ax.set_ylim(0, 60)

# # 显示图形
# plt.grid(True)  # 显示网格，可选
# plt.show()

import matplotlib.pyplot as plt
import custom_tree
from matplotlib.patches import Circle
import numpy as np

# 障碍物和参数设置
circles = [[30, 20, 10], [40, 45, 8]]
step = 5                  # 步长
max_iter = 5000           # 最大迭代次数
pos_init = [0, 0]         # 起点
pos_goal = [90, 60]       # 目标点
threshold_dis = 5         # 目标距离阈值

# 生成随机点（带目标偏向）
def generate_random():
    if np.random.random() < 0.1:  # 10%概率采样目标点
        return pos_goal.copy()
    else:
        return [np.random.uniform(0, 90), np.random.uniform(0, 60)]

# 线段与圆碰撞检测
def line_segment_circle_collision(p1, p2, circle_center, circle_radius):
    line_vec = [p2[0] - p1[0], p2[1] - p1[1]]
    to_center = [circle_center[0] - p1[0], circle_center[1] - p1[1]]
    line_len_sq = line_vec[0]** 2 + line_vec[1]** 2
    t = (to_center[0] * line_vec[0] + to_center[1] * line_vec[1]) / line_len_sq if line_len_sq != 0 else 0
    t = max(0, min(1, t))
    closest_point = [p1[0] + t * line_vec[0], p1[1] + t * line_vec[1]]
    dist_sq = (closest_point[0] - circle_center[0])**2 + (closest_point[1] - circle_center[1])** 2
    return dist_sq <= circle_radius** 2

# 寻找最近节点并生成新节点（带碰撞检测）
def free_collision(random, tree):
    if not tree.nodes:
        return False, None, -1
    
    # 找最近节点
    min_distance = (random[0] - tree.nodes[0].pos[0])**2 + (random[1] - tree.nodes[0].pos[1])** 2
    nearest_node = tree.nodes[0]
    node_id = 0
    for i in range(len(tree.nodes)):
        distance = (random[0] - tree.nodes[i].pos[0])**2 + (random[1] - tree.nodes[i].pos[1])** 2
        if distance < min_distance:
            min_distance = distance
            nearest_node = tree.nodes[i]
            node_id = i
    
    # 生成新节点位置
    dx = random[0] - nearest_node.pos[0]
    dy = random[1] - nearest_node.pos[1]
    dist = np.sqrt(dx** 2 + dy** 2)
    if dist == 0:
        return False, None, -1
    
    dir_x, dir_y = dx / dist, dy / dist
    new_pos = [nearest_node.pos[0] + step * dir_x, nearest_node.pos[1] + step * dir_y]
    
    # 检测线段碰撞（最近节点到新节点）
    collision = False
    for circle in circles:
        if line_segment_circle_collision(nearest_node.pos, new_pos, circle[:2], circle[2]):
            collision = True
            break
    
    new_node = custom_tree.Node(pos=new_pos, parent_id=node_id)
    return not collision, new_node, node_id

# 初始化树和绘图
root = custom_tree.Node(pos=pos_init)
tree = custom_tree.Tree()
tree.add_node(root)

# 创建画布并设置参数
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, 90)
ax.set_ylim(0, 60)
plt.grid(True)
plt.title("RRT")

# 绘制障碍物、起点和终点
for circle in circles:
    ax.add_patch(Circle(circle[:2], circle[2], fill=True, color='blue'))
ax.plot(pos_init[0], pos_init[1], 'go', markersize=8, label='起点')
ax.plot(pos_goal[0], pos_goal[1], 'ro', markersize=8, label='终点')
plt.legend()

# 主循环（带动态刷新）
find_path = False
for i in range(max_iter):
    random = generate_random()
    valid, new_node, parent_id = free_collision(random, tree)
    
    if not valid:
        continue
    
    # 添加新节点并绘制线段
    tree.add_node(new_node)
    parent_pos = tree.nodes[parent_id].pos
    ax.plot(
        [parent_pos[0], new_node.pos[0]],
        [parent_pos[1], new_node.pos[1]],
        'k-', alpha=0.3, linewidth=1
    )
    ax.plot(new_node.pos[0], new_node.pos[1], 'bx', markersize=3)  # 新节点标记
    
    # 检查是否到达目标
    dist_to_goal = np.sqrt((pos_goal[0] - new_node.pos[0])** 2 + (pos_goal[1] - new_node.pos[1])** 2)
    if dist_to_goal < threshold_dis:
        # 绘制最终路径
        ax.plot(
            [new_node.pos[0], pos_goal[0]],
            [new_node.pos[1], pos_goal[1]],
            'r-', linewidth=2, label='最终路径'
        )
        find_path = True
        break
    
    # 动态刷新（每50次迭代或找到路径时）
    if i % 50 == 0 or find_path:
        plt.pause(0.001)  # 暂停以刷新画面
        if find_path:
            break

# 显示结果
if find_path:
    print(f"成功找到路径！迭代次数：{i+1}")
else:
    print("未找到有效路径，达到最大迭代次数。")

plt.show()