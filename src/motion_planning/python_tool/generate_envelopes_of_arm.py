# pca工具包
from sklearn.decomposition import PCA
from mpl_toolkits.mplot3d import Axes3D

# 画图工具
import matplotlib.colors as mcolors
import matplotlib.cm as cm
import matplotlib.pyplot as pit

# 线性代数工具
import numpy as np

import os

# 旋转矩阵转四元数
from scipy.spatial.transform import Rotation

class EnvelopeGenerator:
    # 三角面数据

    def __init__(self):
        self.vertices = []

        # 圆柱数据
        x = 0
        y = 0
        z = 0
        radius = 0
        length = 0

    # 加载模型数据，现在仅支持obj类型的模型文件
    # 在obj文件格式中，v指代vertex,即三角面
    def load_data(self,file_path=[]):
        for model_file in file_path:
            with open(model_file,"r") as file:
                for line in file:
                    if line.startswith("v "):# 仅匹配实际上开头为'v '的数据，通过空格避免读取vn等面法向量
                        vertex = line.strip().split(' ')[1:]
                        vertex = [float(coord) for coord in vertex]
                        self.vertices.append(vertex)    

    # 删除模型数据
    def clear_data(self):
        self.vertices.clear()

    # 产生包络体
    def generate_envelope(self):
        # 数据从三维降为二维
        data = np.array(self.vertices)
        pca = PCA(n_components=0.95)
        data_pca = pca.fit_transform(data)

        # 得到质心
        origin = np.mean(data,axis=0)
        x = origin[0]
        y = origin[1]
        z = origin[2]

        print(f'center x ',{x})
        print(f'center y ',{y})
        print(f'center z ',{z})

        max_x = np.max(np.abs(data_pca[:, 0]))  # 直接计算第一列的绝对值最大值
        min_x = np.max(np.abs(data_pca[:, 0]))  # 直接计算第一列的绝对值最小值
        max_y = np.max(np.abs(data_pca[:, 1]))  # 直接计算第二列的绝对值最大值
        min_y = np.max(np.abs(data_pca[:, 1]))  # 直接计算第二列的绝对值最小值

        print(f"X 轴绝对值最大分量: {max_x}")
        print(f"Y 轴绝对值最大分量: {max_y}")

        # 获取 PCA 主成分（形状为 (2, 3)）
        components = pca.components_  # 两个 3D 基向量
        v1 = components[0]
        v2 = components[1]
        v1 = v1 / np.linalg.norm(v1)  # 归一化
        v2 = v2 - np.dot(v2, v1) * v1  # 正交化
        v2 = v2 / np.linalg.norm(v2)  # 归一化

        # 生成第三个基向量（通过叉乘确保正交）
        v3 = np.cross(v1, v2)

        # 构造完整的 3D 旋转矩阵（每一列是一个基向量）
        rotation_matrix = np.column_stack((v1, v2, v3))

        # 验证行列式（应为 1 或 -1）
        det = np.linalg.det(rotation_matrix)
        if det < 0:
            # 如果行列式为负，翻转一个轴
            rotation_matrix[:, 2] *= -1

        # 转换为四元数
        r = Rotation.from_matrix(rotation_matrix)
        quaternion = r.as_quat()  # [x, y, z, w]

        print(f"四元数 (x, y, z, w): {quaternion}")

if __name__ == '__main__':
    # 读取文件夹下的所有后缀名为'.obj'文件，并储存到列表中
    # 指定搜索目录
    specify_search_directory = '/home/up/robotics-manipulation/src/mujoco_resource/assets'
    model_files= []

    # model_files.append('/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_0.obj')
    # model_files.append('/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_1.obj')
    # model_files.append('/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_2.obj')
    model_files.append('/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_1.obj')

    # try:
    #     if not os.path.exists(specify_search_directory):
    #         raise FileNotFoundError(f"The directory that provided isn't exists",{specify_search_directory})
        
    #     for root,_,files in os.walk(specify_search_directory):
    #         for file in files:
    #             if file.lower().endswith(".obj"):
    #                 model_files.append(os.path.join(root,file))
    #         print("In the ollow,there are files that we searched : ")
    #         for file in model_files:
    #             print(file)
    # except PermissionError:
    #     print(f"User have no enough permission to access",{specify_search_directory})
    # except Exception as e:
    #     print(f"Error: ",{e})

    eg = EnvelopeGenerator()
    eg.load_data(file_path=model_files)
    eg.generate_envelope()
    eg.clear_data()

    
        
