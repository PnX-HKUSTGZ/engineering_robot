import numpy as np
from stl import mesh # numpy-stl
import open3d as o3d # Open3D
import os

def stl_to_pcd(stl_file_path, pcd_file_path, extract_unique_vertices=True, sample_density=0, verbose=True):
    """
    将 STL 文件转换为 PCD 文件。

    参数:
    stl_file_path (str): 输入的 STL 文件路径。
    pcd_file_path (str): 输出的 PCD 文件路径。
    extract_unique_vertices (bool):
        True: 只提取唯一的顶点作为点云。
        False: 提取每个三角形的所有顶点（会有重复）。
    sample_density (int):
        0 (默认): 不进行表面采样，直接使用 STL 的顶点（根据 extract_unique_vertices）。
        >0 : 在 STL 表面上采样指定数量的点以生成更密集的点云。
             此时，extract_unique_vertices 参数会被忽略，因为采样是基于整个网格的。
    verbose (bool): 是否打印详细信息。

    返回:
    bool: 如果转换成功则返回 True，否则返回 False。
    """
    if not os.path.exists(stl_file_path):
        if verbose:
            print(f"错误：STL 文件未找到于 '{stl_file_path}'")
        return False

    try:
        # 1. 加载 STL 文件
        stl_mesh = mesh.Mesh.from_file(stl_file_path)
        if verbose:
            print(f"成功加载 STL 文件: '{stl_file_path}'")
            print(f"包含 {len(stl_mesh.vectors)} 个三角形。")

    except Exception as e:
        if verbose:
            print(f"错误：无法读取或解析 STL 文件 '{stl_file_path}': {e}")
        return False

    # 创建 Open3D 点云对象
    pcd = o3d.geometry.PointCloud()

    if sample_density > 0:
        if verbose:
            print(f"将在 STL 表面采样 {sample_density} 个点...")
        # Open3D 可以直接从其 TriangleMesh 对象进行采样
        # 首先将 numpy-stl 的 mesh 转换为 Open3D 的 TriangleMesh
        o3d_mesh = o3d.geometry.TriangleMesh()
        o3d_mesh.vertices = o3d.utility.Vector3dVector(stl_mesh.vectors.reshape(-1, 3))
        # numpy-stl 的 vectors 是 (N, 3, 3)，reshape 后是 (N*3, 3)
        # 我们需要为 Open3D TriangleMesh 创建三角形索引
        num_triangles = len(stl_mesh.vectors)
        triangles_indices = np.arange(num_triangles * 3).reshape(num_triangles, 3)
        o3d_mesh.triangles = o3d.utility.Vector3iVector(triangles_indices)

        # 清理和预处理网格（可选但推荐，特别是对于采样）
        o3d_mesh.compute_vertex_normals() # 计算法线，有助于某些采样方法
        o3d_mesh.remove_degenerate_triangles()
        o3d_mesh.remove_duplicated_vertices()
        o3d_mesh.remove_duplicated_triangles()
        o3d_mesh.remove_non_manifold_edges()

        if not o3d_mesh.has_triangles():
             if verbose:
                print("错误：转换到 Open3D mesh 后没有三角形，无法采样。")
             return False

        # 从网格表面采样点云
        # 'poisson_disk_sampling' 通常能产生更均匀的分布
        # 'sample_points_uniformly' 也是一个选项
        try:
            pcd = o3d_mesh.sample_points_poisson_disk(number_of_points=sample_density, init_factor=5)
            # 或者 pcd = o3d_mesh.sample_points_uniformly(number_of_points=sample_density)
            if verbose:
                print(f"成功采样 {len(pcd.points)} 个点。")
        except Exception as e:
            if verbose:
                print(f"错误：在网格上采样点失败: {e}")
            return False

    else:
        # 直接使用 STL 文件中的顶点
        if extract_unique_vertices:
            # 获取唯一的顶点
            vertices = np.unique(stl_mesh.vectors.reshape(-1, 3), axis=0)
            if verbose:
                print(f"提取了 {len(vertices)} 个唯一的顶点。")
        else:
            # 获取所有顶点 (包括重复)
            vertices = stl_mesh.vectors.reshape(-1, 3)
            if verbose:
                print(f"提取了 {len(vertices)} 个顶点 (包括重复)。")

        if vertices.size == 0:
            if verbose:
                print("错误：从 STL 文件中未提取到顶点。")
            return False

        # 将 NumPy 顶点数组转换为 Open3D Vector3dVector
        pcd.points = o3d.utility.Vector3dVector(vertices)

    # 3. 保存为 PCD 文件
    try:
        # Open3D 默认以二进制格式保存 PCD，这通常更好
        # o3d.io.write_point_cloud(pcd_file_path, pcd, write_ascii=False)
        o3d.io.write_point_cloud(pcd_file_path, pcd, write_ascii=False) # 默认就是 binary
        if verbose:
            print(f"点云已成功保存为 PCD 文件: '{pcd_file_path}' (包含 {len(pcd.points)} 个点)")
        return True
    except Exception as e:
        if verbose:
            print(f"错误：保存 PCD 文件 '{pcd_file_path}' 失败: {e}")
        return False

if __name__ == "__main__":
    # --- 配置参数 ---
    input_stl_file = "combined_model.stl"  # <--- 修改为你的STL文件名
    output_pcd_file_unique = "output_unique_vertices.pcd"
    output_pcd_file_all = "output_all_vertices.pcd"
    output_pcd_file_sampled = "output_sampled_surface.pcd"

    # --- 1. 创建一个虚拟的 STL 文件用于演示 ---
    if not os.path.exists(input_stl_file):
        print(f"警告: '{input_stl_file}' 未找到。正在创建一个虚拟的立方体 STL 文件用于演示。")
        verts = np.array([
            [0,0,0], [1,0,0], [1,1,0], [0,1,0],
            [0,0,1], [1,0,1], [1,1,1], [0,1,1]])
        faces_indices = np.array([
            [0,3,1],[1,3,2], [0,1,5],[0,5,4], [0,4,7],[0,7,3],
            [1,2,6],[1,6,5], [2,3,7],[2,7,6], [4,5,6],[4,6,7]])
        cube_data = np.zeros(len(faces_indices), dtype=mesh.Mesh.dtype)
        for i, f_indices in enumerate(faces_indices):
            cube_data['vectors'][i] = verts[f_indices]
        cube_mesh = mesh.Mesh(cube_data)
        cube_mesh.save(input_stl_file)
        print(f"虚拟文件 '{input_stl_file}' 已创建。")
    # --- 虚拟文件创建结束 ---

    print("\n--- 示例 1: 提取唯一顶点 ---")
    if stl_to_pcd(input_stl_file, output_pcd_file_unique, extract_unique_vertices=True, sample_density=0):
        # 你可以用 pcl_viewer output_unique_vertices.pcd 或 CloudCompare 打开查看
        pass

    print("\n--- 示例 2: 提取所有顶点 (包括重复) ---")
    if stl_to_pcd(input_stl_file, output_pcd_file_all, extract_unique_vertices=False, sample_density=0):
        pass

    print("\n--- 示例 3: 从表面采样生成密集点云 ---")
    # 对于虚拟立方体，10000个点可能有点多，但用于演示
    num_sample_points = 10000
    if stl_to_pcd(input_stl_file, output_pcd_file_sampled, sample_density=num_sample_points):
        # 这个文件应该包含大约 num_sample_points 个点，分布在立方体表面
        pass

    # --- 清理虚拟文件 (可选) ---
    # if input_stl_file == "your_model.stl" and os.path.exists(input_stl_file): # 确保只删除我们创建的虚拟文件
    #     os.remove(input_stl_file)
    #     print(f"\n已删除虚拟文件: '{input_stl_file}'")