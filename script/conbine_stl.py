import numpy as np
from stl import mesh # numpy-stl library
import os

def load_stl_mesh(stl_file_path):
    """
    加载 STL 文件并返回其 mesh 对象。

    参数:
    stl_file_path (str): STL 文件的路径。

    返回:
    stl.mesh.Mesh: 加载的 mesh 对象，如果失败则返回 None。
    """
    if not os.path.exists(stl_file_path):
        print(f"错误：文件 '{stl_file_path}' 未找到。")
        return None
    try:
        loaded_mesh = mesh.Mesh.from_file(stl_file_path)
        print(f"成功加载: '{stl_file_path}' (包含 {len(loaded_mesh.vectors)} 个三角形)")
        return loaded_mesh
    except Exception as e:
        print(f"错误：无法读取或解析 STL 文件 '{stl_file_path}': {e}")
        return None

def combine_stl_files(stl_file_path1, stl_file_path2, output_stl_filename):
    """
    合并两个STL文件到一个新的STL文件中。

    参数:
    stl_file_path1 (str): 第一个STL文件的路径。
    stl_file_path2 (str): 第二个STL文件的路径。
    output_stl_filename (str): 输出的合并后的STL文件名。

    返回:
    bool: 如果合并成功并保存则返回 True，否则返回 False。
    """
    # 1. 加载第一个STL文件
    mesh1 = load_stl_mesh(stl_file_path1)
    if mesh1 is None:
        return False

    # 2. 加载第二个STL文件
    mesh2 = load_stl_mesh(stl_file_path2)
    if mesh2 is None:
        return False

    # 3. 合并 mesh 数据
    # stl.mesh.Mesh 对象的数据存储在 .data 属性中，它是一个 NumPy 结构化数组。
    # 我们可以直接连接这两个数组。
    # .data 包含 'vectors' (顶点) 和 'attr' (属性，通常为0) 字段。
    # .vectors 是 (N, 3, 3) 形状的数组，包含每个三角形的三个顶点。
    # .normals 是 (N, 3) 形状的数组，包含每个三角形的法向量。
    # .attr 是 (N,) 形状的数组。

    # 创建一个新的 combined_data 数组，其大小足以容纳两个mesh的三角形
    num_triangles1 = len(mesh1.vectors)
    num_triangles2 = len(mesh2.vectors)
    total_triangles = num_triangles1 + num_triangles2

    # 初始化 combined_data
    # 使用 mesh1.data.dtype 来确保数据类型一致
    combined_data = np.zeros(total_triangles, dtype=mesh1.data.dtype)

    # 复制 mesh1 的数据
    combined_data[0:num_triangles1] = mesh1.data

    # 复制 mesh2 的数据
    combined_data[num_triangles1:total_triangles] = mesh2.data

    # 4. 从合并后的数据创建新的 mesh 对象
    # remove_empty_areas=False 是为了避免意外移除我们刚刚合并的区域
    combined_mesh = mesh.Mesh(combined_data, remove_empty_areas=False)

    # 5. 保存合并后的 mesh 到新的STL文件
    try:
        combined_mesh.save(output_stl_filename)
        print(f"合并后的STL文件已保存为 '{output_stl_filename}' (总共 {total_triangles} 个三角形)")
        return True
    except Exception as e:
        print(f"错误：保存合并后的STL文件 '{output_stl_filename}' 失败: {e}")
        return False

if __name__ == "__main__":
    # --- 创建两个虚拟的STL文件用于演示 ---
    # 文件1: 一个简单的立方体

    stl_file1="filled_contour_.stl"
    stl_file2="filled_contour.stl"
    # --- 虚拟文件创建结束 ---

    # 指定你的STL文件路径和输出文件名
    # stl_file1 = "path/to/your/first_model.stl"  # 替换为你的文件路径
    # stl_file2 = "path/to/your/second_model.stl" # 替换为你的文件路径
    output_stl = "combined_model.stl"

    if combine_stl_files(stl_file1, stl_file2, output_stl):
        print("STL文件合并成功！")
        # 可选: 删除虚拟文件
        # os.remove(stl_file1)
        # os.remove(stl_file2)
    else:
        print("STL文件合并失败。")