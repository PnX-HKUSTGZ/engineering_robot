#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp_nl.h> // IterativeClosestPointNonLinear, also includes IterativeClosestPointWithNormals
#include <pcl/features/normal_3d_omp.h> // For normal estimation (OMP version for speed)
#include <pcl/filters/filter.h>       // For removeNaNFromPointCloud (if normals have NaN)
#include <pcl/visualization/pcl_visualizer.h> // Optional for visualization
#include <Eigen/Core>

// 定义点类型
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointNormal PointNormal; // 包含 x, y, z 和 normal_x, normal_y, normal_z, curvature
typedef pcl::PointXYZINormal PointXYZINormal; // 包含 x, y, z, intensity 和法向量及曲率

int main() {
    // 1. 加载或创建点云
    pcl::PointCloud<PointXYZ>::Ptr cloud_source(new pcl::PointCloud<PointXYZ>);
    pcl::PointCloud<PointXYZ>::Ptr cloud_target_raw(new pcl::PointCloud<PointXYZ>); // 目标点云(仅XYZ)

    // 假设你已经从PCD文件加载了数据
    // if (pcl::io::loadPCDFile<PointXYZ>("source.pcd", *cloud_source) == -1 ||
    //     pcl::io::loadPCDFile<PointXYZ>("target.pcd", *cloud_target_raw) == -1) {
    //     PCL_ERROR("Couldn't read file\n");
    //     return (-1);
    // }

    // --- 为了演示，创建简单的点云数据 ---
    cloud_source->width = 500; cloud_source->height = 1; cloud_source->is_dense = false;
    cloud_source->points.resize(cloud_source->width * cloud_source->height);
    for (size_t i = 0; i < cloud_source->points.size(); ++i) {
        cloud_source->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_source->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_source->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    // 创建一个稍微变换过的目标点云
    *cloud_target_raw = *cloud_source;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.5f, 0.0f, 0.0f; // 平移
    transform.rotate(Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitZ())); // 绕Z轴旋转
    pcl::transformPointCloud(*cloud_target_raw, *cloud_target_raw, transform.inverse()); // 将变换应用到目标上，ICP会尝试找回这个变换
    // --- 演示数据创建结束 ---


    // 2. 为目标点云计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_target_raw);
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(20.0); // 重要: 根据你的点云密度调整这个半径!
    // 或者 ne.setKSearch(20);
    ne.compute(*target_normals);

    // 将XYZ点云和法向量合并到 PointNormal (或 PointXYZINormal) 点云中
    pcl::PointCloud<PointNormal>::Ptr cloud_target_with_normals(new pcl::PointCloud<PointNormal>);
    pcl::concatenateFields(*cloud_target_raw, *target_normals, *cloud_target_with_normals);

    // 检查法向量是否有效 (可选，但推荐)
    // 移除包含NaN法线的点，因为它们会导致ICP失败
    std::vector<int> indices_nan;
    pcl::removeNaNNormalsFromPointCloud(*cloud_target_with_normals, *cloud_target_with_normals, indices_nan);
    if (indices_nan.size() > 0) {
        std::cout << "Removed " << indices_nan.size() << " points with NaN normals from target." << std::endl;
    }
    if (cloud_target_with_normals->empty()){
        PCL_ERROR("Target cloud with normals is empty after NaN removal or normal computation failed.\n");
        return -1;
    }


    // 3. 初始化 IterativeClosestPointWithNormals
    // 源点云类型是 PointXYZ, 目标点云类型是 PointNormal
    pcl::IterativeClosestPointWithNormals<PointXYZ, PointNormal> icp;

    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target_with_normals);

    // 设置ICP参数 (这些参数对结果影响很大，需要根据具体数据调整)
    icp.setMaxCorrespondenceDistance(50.0);    // 最大对应距离 (m 或你的点云单位)
                                             // 点对如果距离大于此值则被忽略
    icp.setTransformationEpsilon(1e-9);       // 两次迭代之间允许的最大转换差异 (停止条件)
    icp.setEuclideanFitnessEpsilon(1e-6);     // 均方误差（MSE）收敛阈值 (停止条件)
    icp.setMaximumIterations(100);             // 最大迭代次数

    // 可选: 设置点表示方式，特别是对于Point-to-Plane，使用PointToPlaneLLS
    // pcl::registration::TransformationEstimationPointToPlaneLLS<PointXYZ, PointNormal>::Ptr trans_lls (new pcl::registration::TransformationEstimationPointToPlaneLLS<PointXYZ, PointNormal>);
    // icp.setTransformationEstimation(trans_lls);
    // 注意: IterativeClosestPointWithNormals 默认内部会使用合适的点到面误差度量。
    // 上面这步显式设置通常不是必需的，除非你想用特定版本的 PointToPlane 估计器。

    std::cout << "Starting Point-to-Plane ICP alignment..." << std::endl;

    // 4. 执行配准
    pcl::PointCloud<PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<PointXYZ>);
    icp.align(*cloud_aligned); // cloud_aligned 是配准后的源点云

    // 5. 获取结果
    if (icp.hasConverged()) {
        std::cout << "ICP has converged." << std::endl;
        std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl; // 越小越好
        Eigen::Matrix4f final_transformation = icp.getFinalTransformation();
        std::cout << "Final transformation:" << std::endl << final_transformation << std::endl;

        // 可选：可视化
        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ICP Point-to-Plane Demo"));
        // viewer->setBackgroundColor(0, 0, 0);
        // // 源点云 (红色)
        // pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> source_color(cloud_source, 255, 0, 0);
        // viewer->addPointCloud<PointXYZ>(cloud_source, source_color, "source_cloud");
        // // 目标点云 (绿色) - 注意这里我们显示原始的XYZ点云，但ICP内部用了带法线的
        // pcl::visualization::PointCloudColorHandlerCustom<PointNormal> target_color(cloud_target_with_normals, 0, 255, 0);
        // viewer->addPointCloud<PointNormal>(cloud_target_with_normals, target_color, "target_cloud_with_normals");
        // // 配准后的源点云 (蓝色)
        // pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> aligned_color(cloud_aligned, 0, 0, 255);
        // viewer->addPointCloud<PointXYZ>(cloud_aligned, aligned_color, "aligned_cloud");
        // viewer->addCoordinateSystem(1.0);
        // viewer->initCameraParameters();
        // while (!viewer->wasStopped()) {
        //     viewer->spinOnce(100);
        // }

    } else {
        PCL_ERROR("ICP did not converge.\n");
        return -1;
    }

    return 0;
}