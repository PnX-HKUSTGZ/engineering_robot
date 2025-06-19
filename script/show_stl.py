import numpy as np
from stl import mesh # numpy-stl library
import os

def extract_vertices_from_stl(stl_file_path):
    """
    从 STL 文件中读取所有唯一的顶点坐标。

    参数:
    stl_file_path (str): STL 文件的路径。

    返回:
    numpy.ndarray: 一个 N x 3 的 NumPy 数组，包含所有唯一的顶点坐标，
                   如果文件不存在或读取失败则返回 None。
    """
    if not os.path.exists(stl_file_path):
        print(f"错误：文件未找到于 '{stl_file_path}'")
        return None

    try:
        # 加载 STL 文件
        # from_file 会自动检测是 ASCII 还是二进制格式
        your_mesh = mesh.Mesh.from_file(stl_file_path)
    except Exception as e:
        print(f"错误：无法读取或解析 STL 文件 '{stl_file_path}': {e}")
        return None

    # mesh.Mesh 对象有一个 'vectors' 属性，它是一个 (N, 3, 3) 的 NumPy 数组。
    # N 是三角形的数量。
    # 每个三角形有 3 个顶点。
    # 每个顶点有 3 个坐标 (x, y, z)。

    # 将所有顶点收集到一个列表中 (N*3, 3)
    # your_mesh.v0, your_mesh.v1, your_mesh.v2 分别是每个三角形的第一个、第二个、第三个顶点
    # all_vertices = np.concatenate((your_mesh.v0, your_mesh.v1, your_mesh.v2), axis=0)
    # 更简洁的方式是直接使用 reshape on vectors:
    all_vertices_with_duplicates = your_mesh.vectors.reshape(-1, 3)

    # STL 文件中的顶点可能会重复（因为一个顶点可以被多个三角形共享）
    # 使用 np.unique 找到唯一的顶点
    unique_vertices = np.unique(all_vertices_with_duplicates, axis=0)

    return unique_vertices

if __name__ == "__main__":
    # 请将 'your_model.stl' 替换为你的 STL 文件路径
    stl_file = 'combined_model.stl'

    # --- 为了测试，如果文件不存在，我们可以创建一个简单的虚拟 STL 文件 ---
    if not os.path.exists(stl_file):
        print(f"警告: '{stl_file}' 未找到。正在创建一个虚拟的立方体 STL 文件用于演示。")
        # 定义一个立方体的8个顶点
        verts = np.array([
            [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
            [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]])
        # 定义立方体的12个面（每个面2个三角形）
        faces_indices = np.array([
            [0, 3, 1], [1, 3, 2], # 底面
            [4, 5, 7], [5, 6, 7], # 顶面
            [0, 1, 5], [0, 5, 4], # 前面
            [2, 3, 7], [2, 7, 6], # 后面
            [0, 4, 7], [0, 7, 3], # 左面
            [1, 2, 6], [1, 6, 5]  # 右面
        ])
        # 创建 mesh 数据
        cube_data = np.zeros(len(faces_indices), dtype=mesh.Mesh.dtype)
        for i, f_indices in enumerate(faces_indices):
            cube_data['vectors'][i] = verts[f_indices]

        cube_mesh = mesh.Mesh(cube_data)
        cube_mesh.save(stl_file)
        print(f"虚拟文件 '{stl_file}' 已创建。")
    # --- 虚拟 STL 文件创建结束 ---

    point_cloud = extract_vertices_from_stl(stl_file)

    if point_cloud is not None:
        print(f"\n从 '{stl_file}' 中提取到 {len(point_cloud)} 个唯一的顶点:")
        for i, point in enumerate(point_cloud):
            # 格式化输出，保留小数点后几位可以使输出更整洁
            print(f"点 {i+1:>4}: X={point[0]:>10.6f}, Y={point[1]:>10.6f}, Z={point[2]:>10.6f}")

        # 如果需要，可以将点云保存到文本文件
        # output_txt_file = "extracted_point_cloud.txt"
        # np.savetxt(output_txt_file, point_cloud, fmt='%.6f %.6f %.6f')
        # print(f"\n点云数据已保存到 '{output_txt_file}'")

        # 或者保存为更常见的 .xyz 格式
        # output_xyz_file = "extracted_point_cloud.xyz"
        # with open(output_xyz_file, 'w') as f:
        #     for point in point_cloud:
        #         f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
        # print(f"点云数据已保存到 '{output_xyz_file}'")