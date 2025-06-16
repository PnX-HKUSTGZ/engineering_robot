# sensor_drivers

## camera_driver

话题：

1. `/sensor/camera/images` 
    类型 `sensor_msgs/msg/image`，发布摄像头图像
2. tf2
    ```
    to_map.header.frame_id="robot_base_link";
    to_map.child_frame_id="sensor/camera";
    to_map.header.stamp=this->now();
    to_map.transform.rotation.w=config["object_pos"]["camera"]["rotate"]["w"].as<double>();
    to_map.transform.rotation.x=config["object_pos"]["camera"]["rotate"]["x"].as<double>();
    to_map.transform.rotation.y=config["object_pos"]["camera"]["rotate"]["y"].as<double>();
    to_map.transform.rotation.z=config["object_pos"]["camera"]["rotate"]["z"].as<double>();
    to_map.transform.translation.x=config["object_pos"]["camera"]["translation"]["x"].as<double>();
    to_map.transform.translation.y=config["object_pos"]["camera"]["translation"]["y"].as<double>();
    to_map.transform.translation.z=config["object_pos"]["camera"]["translation"]["z"].as<double>();
    ```
## mid360_driver

话题：

1. `sensor/mid360/point_cloud_all`
    类型 `sensor_msgs/msg/point_cloud2`，发布一定时间内累积的点云
    累积时间定义在`config.yaml`中
    `buffertime=rclcpp::Duration(config["mid_360"]["buffertime"].as<std::vector<int>>()[0],config["mid_360"]["buffertime"].as<std::vector<int>>()[1])`
2. `sensor/mid360/point_cloud`
    类型 `sensor_msgs/msg/point_cloud2`，发布点云，mid360传过来什么就发什么
3. `sensor/mid360/imu`
    类型 `sensor_msgs/msg/imu`，发布imu数据
4. tf2
    ```
    t.header.stamp =this->now();
    t.header.frame_id = "map";
    t.child_frame_id = "sensor/mid360";
    t.transform.translation.x = config["object_pos"]["mid360"]["translation"]["x"].as<double>();
    t.transform.translation.y = config["object_pos"]["mid360"]["translation"]["y"].as<double>();
    t.transform.translation.z = config["object_pos"]["mid360"]["translation"]["z"].as<double>();
    t.transform.rotation.x = config["object_pos"]["mid360"]["rotate"]["x"].as<double>();
    t.transform.rotation.y = config["object_pos"]["mid360"]["rotate"]["y"].as<double>();
    t.transform.rotation.z = config["object_pos"]["mid360"]["rotate"]["z"].as<double>();
    t.transform.rotation.w = config["object_pos"]["mid360"]["rotate"]["w"].as<double>();
    tf_static_transform_broadcaster->sendTransform(t);
    ```
    这个要改！

## RealSenseDriver

话题：
1. `sensor/RealSense/image`
    类型 `sensor_msgs/msg/image`，发布摄像头图像
2. `sensor/RealSense/point_cloud`
    类型 `sensor_msgs/msg/point_cloud2`，发布点云