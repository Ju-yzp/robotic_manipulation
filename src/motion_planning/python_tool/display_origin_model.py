import trimesh
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

def load_and_merge_objs(obj_files):
    """加载多个OBJ文件并合并顶点和三角面"""
    all_vertices = []
    all_faces = []
    vertex_offset = 0  # 用于调整不同模型的面索引（避免索引冲突）
    
    for file in obj_files:
        if not os.path.exists(file):
            print(f"警告：文件不存在 - {file}，已跳过")
            continue
            
        try:
            mesh = trimesh.load(file)
            num_vertices = len(mesh.vertices)
            
            # 保存当前模型的顶点（后续会合并）
            all_vertices.append(mesh.vertices)
            
            # 调整面索引（加上偏移量，确保不同模型的面索引不冲突）
            adjusted_faces = mesh.faces + vertex_offset
            all_faces.append(adjusted_faces)
            
            # 更新偏移量（为下一个模型做准备）
            vertex_offset += num_vertices
            
            print(f"已加载：{os.path.basename(file)} "
                  f"(顶点数: {num_vertices}, 面数: {len(mesh.faces)})")
        except Exception as e:
            print(f"加载 {file} 失败：{e}，已跳过")
    
    if not all_vertices:
        raise ValueError("没有成功加载任何OBJ文件，请检查文件路径")
    
    # 合并所有顶点和三角面
    merged_vertices = np.concatenate(all_vertices, axis=0)
    merged_faces = np.concatenate(all_faces, axis=0)
    
    print(f"\n合并完成 - 总顶点数: {len(merged_vertices)}, 总面数: {len(merged_faces)}")
    return merged_vertices, merged_faces

def visualize_merged_meshes(vertices, faces):
    """可视化合并后的所有三角面"""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制所有三角面的边缘
    for i, face in enumerate(faces):
        # 每1000个面打印一次进度（大型模型时方便观察）
        if i % 1000 == 0 and i > 0:
            print(f"已绘制 {i}/{len(faces)} 个面...")
            
        v0, v1, v2 = vertices[face]
        # 绘制三角形的三条边（形成闭合三角形）
        ax.plot(
            [v0[0], v1[0], v2[0], v0[0]],
            [v0[1], v1[1], v2[1], v0[1]],
            [v0[2], v1[2], v2[2], v0[2]],
            color='skyblue', linewidth=0.6, alpha=0.8
        )
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Merged OBJ Models (Total faces: {len(faces)})')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 在这里添加你的多个OBJ文件路径
    obj_files = [
        "/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_0.obj",
        "/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_1.obj",
        "/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_2.obj",
        "/home/up/robotics-manipulation/src/mujoco_resource/assets/forearm_3.obj",
        # 添加更多OBJ文件路径
        # "/path/to/second.obj",
        # "/path/to/third.obj"
    ]
    
    # 加载并合并模型
    merged_vertices, merged_faces = load_and_merge_objs(obj_files)
    
    # 可视化所有三角面
    visualize_merged_meshes(merged_vertices, merged_faces)
