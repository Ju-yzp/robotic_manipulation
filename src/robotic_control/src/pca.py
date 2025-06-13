import numpy as np
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors as mcolors
import matplotlib.cm as cm

def read_obj_vertices(file_path):
    vertices = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('v '):
                vertex = line.strip().split(' ')[1:]
                vertex = [float(coord) for coord in vertex]
                vertices.append(vertex)
    return vertices

if __name__ == '__main__':
    obj_file_path = '/home/up/rm_mujoco_simulate/src/robotic_control/assets/upperarm_0.obj'
    vertices = read_obj_vertices(obj_file_path)
    
    # 将列表转换为NumPy数组
    data = np.array(vertices)  # 关键修改：转换为NumPy数组
    
    # 数据降维
    pca = PCA(n_components=2)
    data_pca = pca.fit_transform(data)
    
    # 创建颜色映射 - 使用连续颜色而不是离散颜色
    # 使用z坐标值作为颜色依据
    z_vals = data[:, 2]
    norm = mcolors.Normalize(vmin=z_vals.min(), vmax=z_vals.max())
    colormap = cm.viridis  # 可以使用其他颜色映射如'plasma', 'inferno', 'magma', 'cividis'
    
    # 创建图形
    fig = plt.figure(figsize=(14, 7))
    
    # 3D散点图展示原始数据和主成分
    ax = fig.add_subplot(121, projection='3d')
    
    # 使用连续颜色映射
    sc = ax.scatter(data[:, 0], data[:, 1], data[:, 2], 
                   c=z_vals, cmap=colormap, alpha=0.6, label='Original data')
    
    # 添加颜色条
    fig.colorbar(sc, ax=ax, label='Z Value')
    
    origin = np.mean(data, axis=0)
    
    min_x = data_pca[0][0]
    min_y = data_pca[0][1]
    max_x = data_pca[0][0]
    max_y = data_pca[0][1]

    for i in range(0,len(data_pca)-1):
        if data_pca[i][0]  < min_x:
            min_x = data_pca[i][0]
        if data_pca[i][0]  > max_x:
            max_x = data_pca[i][0]
        if data_pca[i][1]  < min_y:
            min_y = data_pca[i][1]
        if data_pca[i][1]  > max_y:
            max_y = data_pca[i][1]

    print("分量一最小值",min_x)
    print("分量一最大值",max_x)
    print("分量二最小值",min_y)
    print("分量二最大值",max_y)
    # 获取主成分
    components = pca.components_
    explained_variance = pca.explained_variance_
    
    # 绘制主成分向量
    for i, (length, vector) in enumerate(zip(explained_variance, components)):
        v = vector * 3 * np.sqrt(length)
        ax.quiver(*origin, *v, color='red', linewidth=2, label=f'PC {i+1}')
        print(f"Principal Component {i+1} Direction (scaled): {v}")
    
    ax.set_title('Original 3D Data with Principal Components')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    # 2D散点图展示投影后的数据
    ax2 = fig.add_subplot(122)
    
    # 使用相同的颜色映射
    sc2 = ax2.scatter(data_pca[:, 0], data_pca[:, 1], 
                     c=z_vals, cmap=colormap, alpha=0.6, label='Projected data')
    
    # 添加颜色条
    fig.colorbar(sc2, ax=ax2, label='Z Value')
    
    ax2.axhline(0, color='black', lw=1)
    ax2.axvline(0, color='black', lw=1)
    ax2.set_title('Data Projected to 2D Principal Component Plane')
    ax2.set_xlabel('Principal Component 1')
    ax2.set_ylabel('Principal Component 2')
    ax2.legend()
    
    
    plt.tight_layout()
    plt.show()

# import numpy as np
# from sklearn.decomposition import PCA
# import xml.etree.ElementTree as ET
# from xml.dom import minidom

# def read_obj_vertices(file_path):
#     vertices = []
#     with open(file_path, 'r') as file:
#         for line in file:
#             if line.startswith('v '):
#                 vertex = line.strip().split(' ')[1:]
#                 vertex = [float(coord) for coord in vertex]
#                 vertices.append(vertex)
#     return vertices

# def calculate_pca_parameters(vertices):
#     """计算PCA参数并返回圆柱参数和旋转四元数"""
#     # 将列表转换为NumPy数组
#     data = np.array(vertices)
    
#     # 数据降维
#     pca = PCA(n_components=2)
#     data_pca = pca.fit_transform(data)
    
#     # 获取主成分
#     components = pca.components_
    
#     # 计算数据在主成分平面上的最大距离
#     min_x, max_x = np.min(data_pca[:, 0]), np.max(data_pca[:, 0])
#     min_y, max_y = np.min(data_pca[:, 1]), np.max(data_pca[:, 1])
    
#     # 计算圆柱的高度和半径
#     height = max_x - min_x  # 假设第一个主成分方向作为高度
#     radius = (max_y - min_y) / 2  # 假设第二个主成分方向作为直径
    
#     # 计算中心点
#     center = np.mean(data, axis=0)
    
#     # 计算旋转四元数
#     # 构建旋转矩阵：将Z轴旋转到第一个主成分方向
#     z_axis = np.array([0, 0, 1])
#     pc1 = components[0]  # 第一个主成分
    
#     # 确保主成分是单位向量
#     pc1 = pc1 / np.linalg.norm(pc1)
    
#     # 计算旋转轴和角度
#     rotation_axis = np.cross(z_axis, pc1)
#     rotation_angle = np.arccos(np.dot(z_axis, pc1))
    
#     # 转换为四元数
#     if np.linalg.norm(rotation_axis) < 1e-10:  # 如果几乎平行
#         quat = [1, 0, 0, 0]  # 单位四元数
#     else:
#         rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
#         quat = [
#             np.cos(rotation_angle/2),
#             rotation_axis[0] * np.sin(rotation_angle/2),
#             rotation_axis[1] * np.sin(rotation_angle/2),
#             rotation_axis[2] * np.sin(rotation_angle/2)
#         ]
    
#     return {
#         'center': center,
#         'height': height,
#         'radius': radius,
#         'quaternion': quat
#     }

# def create_mujoco_cylinder_geom(params, name="pca_cylinder_geom"):
#     """创建作为body子元素的Mujoco圆柱geom元素"""
#     # 创建geom元素
#     geom = ET.Element("geom", type="cylinder", name=name)
    
#     # 设置位置（相对于父body的偏移）
#     geom.set("pos", f"{params['center'][0]:.6f} {params['center'][1]:.6f} {params['center'][2]:.6f}")
    
#     # 设置旋转四元数
#     geom.set("quat", f"{params['quaternion'][0]:.6f} {params['quaternion'][1]:.6f} {params['quaternion'][2]:.6f} {params['quaternion'][3]:.6f}")
    
#     # 设置圆柱尺寸
#     geom.set("size", f"{params['radius']:.6f} {params['height']/2:.6f}")  # Mujoco中cylinder的size参数是[radius, half_height]
    
#     # 设置其他属性
#     geom.set("rgba", "0.7 0.7 0.7 0.5")  # 设置颜色和透明度
    
#     # 格式化XML
#     xml_str = ET.tostring(geom, 'utf-8')
#     reparsed = minidom.parseString(xml_str)
#     pretty_xml = reparsed.toprettyxml(indent="  ")
    
#     return pretty_xml

# if __name__ == '__main__':
#     # 读取OBJ文件
#     obj_file_path = '/home/up/rm_mujoco_simulate/src/robotic_control/assets/upperarm_1.obj'
#     vertices = read_obj_vertices(obj_file_path)
    
#     # 计算PCA参数
#     params = calculate_pca_parameters(vertices)
    
#     # 打印计算结果
#     print("=== 计算结果 ===")
#     print(f"圆柱中心点: {params['center']}")
#     print(f"圆柱高度: {params['height']:.6f}")
#     print(f"圆柱半径: {params['radius']:.6f}")
#     print(f"旋转四元数: {params['quaternion']}")
    
#     # 生成Mujoco XML元素
#     mujoco_xml = create_mujoco_cylinder_geom(params)
#     print("\n=== 生成的Mujoco圆柱geom元素 ===")
#     print(mujoco_xml)

