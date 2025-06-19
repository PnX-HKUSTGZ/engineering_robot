import numpy as np
from stl import mesh
from scipy.spatial import Delaunay
import os

def create_filled_stl_from_contour(contour_points_3d, output_stl_filename):
    """
    根据给定的3D轮廓点（假设在同一平面上），
    通过三角剖分填充轮廓，并生成STL文件。

    参数:
    contour_points_3d (list or np.ndarray): 3D轮廓点的列表或NumPy数组。
                                           每个点是 [x, y, z]。
                                           假设所有点共面，且X坐标相同。
    output_stl_filename (str): 输出的STL文件名。
    """
    points_3d = np.array(contour_points_3d)

    if points_3d.shape[0] < 3:
        print("错误：至少需要3个点来形成一个表面。")
        return
    if points_3d.shape[1] != 3:
        print("错误：每个点必须有3个坐标 (x, y, z)。")
        return

    # 假设所有点的X坐标相同，我们将在YZ平面上进行2D三角剖分
    # 提取 YZ 坐标进行2D三角剖分
    points_2d = points_3d[:, 1:]  # Y和Z坐标

    # 使用 Delaunay 三角剖分来填充2D轮廓
    # 注意: Delaunay 会对点的凸包进行三角剖分。
    # 如果轮廓是凹的，并且你希望精确填充凹形，则需要更复杂的polygon triangulation算法。
    # 对于给定的点，Delaunay 应该能给出一个合理的“填充”。
    try:
        tri = Delaunay(points_2d)
    except Exception as e:
        print(f"错误：Delaunay三角剖分失败: {e}")
        print("请确保点不完全共线或数量足够。")
        return

    # tri.simplices 包含了构成三角形的点的索引 (相对于 points_2d 或 points_3d)
    # 例如，[[0, 1, 2], [1, 3, 2]] 表示由点0,1,2和点1,3,2构成的两个三角形

    num_triangles = tri.simplices.shape[0]
    print(f"生成了 {num_triangles} 个三角形。")

    # 创建STL的mesh数据
    # mesh.Mesh.dtype 定义了STL mesh数据的结构
    stl_mesh_data = np.zeros(num_triangles, dtype=mesh.Mesh.dtype)

    for i, triangle_indices in enumerate(tri.simplices):
        # triangle_indices 是 [idx1, idx2, idx3]，对应于 points_3d 中的行号
        # 我们需要获取这三个点的3D坐标来定义一个三角形面片
        vertex1 = points_3d[triangle_indices[0]]
        vertex2 = points_3d[triangle_indices[1]]
        vertex3 = points_3d[triangle_indices[2]]
        stl_mesh_data['vectors'][i] = np.array([vertex1, vertex2, vertex3])

    # 创建STL mesh对象
    surface_mesh = mesh.Mesh(stl_mesh_data)

    # 保存到STL文件
    try:
        surface_mesh.save(output_stl_filename)
        print(f"STL文件已保存为 '{output_stl_filename}'")
    except Exception as e:
        print(f"错误：保存STL文件失败: {e}")


if __name__ == "__main__":
    # 给定的轮廓点
    contour_vertices = [
      [0.144, 0.0, 0.0455],
    #   [0.144, 0.1, 0.1455],
    #   [0.144, 0.08585786437626905, 0.1455],
      [0.144, 0.0, 0.059642135623730955],
      [0.144, -0.08585786437626905, 0.1455],
      [0.144, -0.1, 0.1455]
    ]

    output_file = "filled_contour_.stl"

    create_filled_stl_from_contour(contour_vertices, output_file)

    # 可选：检查文件是否创建
    if os.path.exists(output_file):
        print(f"文件 '{output_file}' 创建成功。你可以用3D查看器打开它。")

        # 如果你想用之前的代码读取并验证顶点：
        # from your_previous_script import extract_vertices_from_stl # 假设你保存了之前的脚本
        # extracted_points = extract_vertices_from_stl(output_file)
        # if extracted_points is not None:
        #     print(f"\n从生成的 '{output_file}' 中提取到 {len(extracted_points)} 个唯一的顶点:")
        #     for i, point in enumerate(extracted_points):
        #         print(f"点 {i+1:>4}: X={point[0]:>10.6f}, Y={point[1]:>10.6f}, Z={point[2]:>10.6f}")