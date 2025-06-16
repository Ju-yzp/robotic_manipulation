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
    obj_file_path = '/home/up/rm_mujoco_simulate/src/robotic_control/assets/_0.obj'
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
    
    for i in range(0,len(origin)-1):
        print(origin[i])
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


