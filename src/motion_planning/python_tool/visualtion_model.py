import trimesh
import numpy as np
import scipy.optimize as opt
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# ----------------------
# 1. 核心功能函数
# ----------------------
def load_and_merge_objs(obj_files):
    """加载多个OBJ文件并合并顶点数据"""
    all_vertices = []
    total_files = len(obj_files)
    
    for i, obj_file in enumerate(obj_files):
        if not os.path.exists(obj_file):
            print(f"警告：文件不存在 - {obj_file}，已跳过")
            continue
        
        try:
            mesh = trimesh.load(obj_file)
            vertices = mesh.vertices
            all_vertices.append(vertices)
            print(f"已加载 {i+1}/{total_files}：{os.path.basename(obj_file)}（{len(vertices)}个顶点）")
        except Exception as e:
            print(f"加载 {obj_file} 失败：{e}，已跳过")
    
    if not all_vertices:
        raise ValueError("没有成功加载任何OBJ文件，请检查文件路径")
    
    # 合并所有顶点为一个点云
    merged_vertices = np.concatenate(all_vertices, axis=0)
    print(f"\n合并完成，总顶点数：{len(merged_vertices)}")
    return merged_vertices

def preprocess_merged_point_cloud(vertices):
    """预处理合并后的点云（去重+滤波）"""
    # 去重（不同OBJ可能有重复顶点）
    vertices = np.unique(vertices, axis=0)
    print(f"去重后顶点数：{len(vertices)}")
    
    # 统计滤波去除离群点（合并后可能有更多噪声）
    mean = np.mean(vertices, axis=0)
    dist = np.linalg.norm(vertices - mean, axis=1)
    threshold = np.mean(dist) + 2.5 * np.std(dist)  # 放宽阈值适应多模型
    vertices = vertices[dist < threshold]
    print(f"滤波后顶点数：{len(vertices)}")
    return vertices

def ransac_filter_cylinder(points, radius_min=0.02, radius_max=0.2, threshold=0.005):
    """RANSAC筛选圆柱候选点（适应合并后的大数据量）"""
    inliers = []
    best_count = 0
    # 增加迭代次数以适应更大的点云
    for _ in range(2000):
        sample_indices = np.random.choice(len(points), 5, replace=False)
        sample = points[sample_indices]
        
        pca_sample = PCA(n_components=2)
        pca_sample.fit(sample)
        axis_dir = pca_sample.components_[0]
        axis_dir /= np.linalg.norm(axis_dir)
        
        axis_point = sample.mean(axis=0)
        vectors = points - axis_point
        cross = np.cross(vectors, axis_dir)
        dist_to_axis = np.linalg.norm(cross, axis=1)
        
        current_inliers = np.where(
            (dist_to_axis > radius_min - threshold) & 
            (dist_to_axis < radius_max + threshold)
        )[0]
        
        if len(current_inliers) > best_count:
            best_count = len(current_inliers)
            inliers = current_inliers
    
    print(f"筛选出圆柱候选点：{len(inliers)}（占总点数 {len(inliers)/len(points):.1%}）")
    return inliers

def least_squares_refine(points, inliers):
    """最小二乘精修圆柱参数"""
    points_candidate = points[inliers]
    if len(points_candidate) < 50:  # 合并后点云应至少有足够候选点
        raise ValueError("候选点太少，无法拟合（合并后的点云可能不含明显圆柱特征）")
    
    def cylinder_cost(params):
        a, b, c, x0, y0, z0, r = params
        axis_dir = np.array([a, b, c])
        axis_dir = axis_dir / np.linalg.norm(axis_dir)
        axis_point = np.array([x0, y0, z0])
        
        vectors = points_candidate - axis_point
        cross = np.cross(vectors, axis_dir)
        dist_to_axis = np.linalg.norm(cross, axis=1)
        return np.abs(dist_to_axis - r)
    
    pca = PCA(n_components=3)
    pca.fit(points_candidate)
    mean = pca.mean_
    axis_dir_init = pca.components_[0]
    axis_dir_init = axis_dir_init / np.linalg.norm(axis_dir_init)
    
    vectors_init = points_candidate - mean
    dist_init = np.linalg.norm(np.cross(vectors_init, axis_dir_init), axis=1)
    radius_init = np.mean(dist_init)
    
    initial_guess = [
        axis_dir_init[0], axis_dir_init[1], axis_dir_init[2],
        mean[0], mean[1], mean[2], radius_init
    ]
    
    result = opt.least_squares(
        cylinder_cost,
        initial_guess,
        bounds=(
            [-1, -1, -1, -np.inf, -np.inf, -np.inf, 0.001],
            [1, 1, 1, np.inf, np.inf, np.inf, 1.0]
        )
    )
    
    a, b, c, x0, y0, z0, r = result.x
    axis_dir = np.array([a, b, c]) / np.linalg.norm(np.array([a, b, c]))
    axis_point = np.array([x0, y0, z0])
    
    return axis_dir, axis_point, r

def calculate_cylinder_length(points, axis_dir, axis_point):
    """计算圆柱长度（沿轴线方向的投影范围）"""
    vectors = points - axis_point
    projections = np.dot(vectors, axis_dir)  # 点在轴线上的投影值
    min_proj, max_proj = projections.min(), projections.max()
    length = max_proj - min_proj
    return length

def visualize_merged_result(merged_points, axis_dir, axis_point, r, length, inliers):
    """可视化合并点云和拟合结果"""
    inlier_points = merged_points[inliers]
    outlier_points = merged_points[np.setdiff1d(range(len(merged_points)), inliers)]
    
    # 生成圆柱表面点
    if np.abs(axis_dir[2]) < 1 - 1e-6:
        v1 = np.cross(axis_dir, [0, 0, 1])
    else:
        v1 = np.cross(axis_dir, [1, 0, 0])
    v1 = v1 / np.linalg.norm(v1)
    v2 = np.cross(axis_dir, v1)
    v2 = v2 / np.linalg.norm(v2)
    
    theta = np.linspace(0, 2*np.pi, 50)
    z_proj = np.linspace(-length/2, length/2, 20)
    theta, z_proj = np.meshgrid(theta, z_proj)
    
    # 圆柱表面坐标
    x = axis_point[0] + r * np.cos(theta) * v1[0] + r * np.sin(theta) * v2[0] + (z_proj + length/2) * axis_dir[0]
    y = axis_point[1] + r * np.cos(theta) * v1[1] + r * np.sin(theta) * v2[1] + (z_proj + length/2) * axis_dir[1]
    z_coord = axis_point[2] + r * np.cos(theta) * v1[2] + r * np.sin(theta) * v2[2] + (z_proj + length/2) * axis_dir[2]
    
    # 1. 合并点云可视化
    fig1 = plt.figure(figsize=(10, 8))
    ax1 = fig1.add_subplot(111, projection='3d')
    ax1.scatter(merged_points[:,0], merged_points[:,1], merged_points[:,2], 
                c='gray', s=5, alpha=0.6)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title(f'合并后点云（{len(merged_points)}个顶点）')
    plt.show()
    
    # 2. 拟合结果可视化
    fig2 = plt.figure(figsize=(10, 8))
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.scatter(outlier_points[:,0], outlier_points[:,1], outlier_points[:,2], 
                c='gray', s=3, alpha=0.2)
    ax2.scatter(inlier_points[:,0], inlier_points[:,1], inlier_points[:,2], 
                c='red', s=8, alpha=0.8, label='圆柱候选点')
    ax2.plot_surface(x, y, z_coord, color='blue', alpha=0.3)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_title(f'合并点云拟合结果（长度={length:.4f}, 半径={r:.4f}）')
    ax2.legend()
    plt.show()

# ----------------------
# 2. 主流程
# ----------------------
def merge_and_fit(obj_files, visualize=True):
    """合并多个OBJ文件并拟合圆柱体"""
    # 步骤1: 合并所有OBJ的顶点
    merged_vertices = load_and_merge_objs(obj_files)
    
    # 步骤2: 预处理合并后的点云
    processed_points = preprocess_merged_point_cloud(merged_vertices)
    if len(processed_points) < 100:
        raise ValueError("合并并预处理后点云数量过少，无法进行有效拟合")
    
    # 步骤3: RANSAC筛选圆柱候选点
    inliers = ransac_filter_cylinder(processed_points)
    if len(inliers) < 50:
        raise ValueError("筛选出的圆柱候选点不足，可能合并后的点云不含明显圆柱特征")
    
    # 步骤4: 最小二乘精修参数
    axis_dir, axis_point, radius = least_squares_refine(processed_points, inliers)
    
    # 步骤5: 计算圆柱长度
    length = calculate_cylinder_length(processed_points, axis_dir, axis_point)
    
    # 输出结果
    # print("\n" + "="*50)
    # print("合并点云拟合结果")
    # print("="*50)
    print(f"cylinder length: {length:.4f}")
    print(f"cylinder: {radius:.4f}")
    print(f"rotation vector: {axis_dir.round(4)}")
    # print("="*50)
    
    # 可视化
    if visualize:
        visualize_merged_result(processed_points, axis_dir, axis_point, radius, length, inliers)
    
    return {
        "length": length,
        "radius": radius,
        "axis_dir": axis_dir,
        "axis_point": axis_point,
        "total_points": len(processed_points)
    }

# ----------------------
# 3. 运行入口
# ----------------------
if __name__ == "__main__":
    # 替换为你的多个OBJ文件路径列表
    obj_files = [
        "/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_0.obj",
        "/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_1.obj",  # 示例文件1
        "/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_2.obj",
        "/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_3.obj"  # 示例文件2
        # 可继续添加更多文件...
    ]
    
    # 执行合并拟合（visualize=False 可关闭可视化）
    merge_and_fit(obj_files, visualize=True)



