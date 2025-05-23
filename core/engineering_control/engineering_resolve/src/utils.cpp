#include "engineering_resolve/engineering_resolve.hpp"

namespace Engineering_robot_Pnx {
/**
 * @brief 将欧拉角转换为四元数
 * @param roll 绕X轴的旋转角度(弧度)
 * @param pitch 绕Y轴的旋转角度(弧度)
 * @param yaw 绕Z轴的旋转角度(弧度)
 * @return 对应的四元数 [w, x, y, z]
 */
std::vector<double> eulerToQuaternion(double roll, double pitch, double yaw) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    std::vector<double> q(4);
    q[0] = cr * cp * cy + sr * sp * sy;  // w
    q[1] = sr * cp * cy - cr * sp * sy;  // x
    q[2] = cr * sp * cy + sr * cp * sy;  // y
    q[3] = cr * cp * sy - sr * sp * cy;  // z

    return q;
}

/**
 * @brief 将欧拉角转换为四元数(向量输入版本)
 * @param euler 欧拉角向量 [roll, pitch, yaw] (弧度)
 * @return 对应的四元数 [w, x, y, z]
 * @throws std::invalid_argument 如果输入向量大小不为3
 */
std::vector<double> eulerToQuaternion(const std::vector<double>& euler) {
    if (euler.size() != 3) {
        throw std::invalid_argument("Euler angles vector must have exactly 3 elements");
    }
    return eulerToQuaternion(euler[0], euler[1], euler[2]);
}

/**
 * @brief 将四元数转换为欧拉角
 * @param q 四元数 [w, x, y, z]
 * @return 对应的欧拉角 [roll, pitch, yaw] (弧度)
 */
std::vector<double> quaternionToEuler(const std::vector<double>& q) {
    std::vector<double> angles(3);

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

void doPointTransform(
    const geometry_msgs::msg::Point &data_in,
    geometry_msgs::msg::Point &data_out,
    const geometry_msgs::msg::TransformStamped &transform
)
{
    // 1. Convert geometry_msgs::Point to tf2::Vector3
    //    tf2::Vector3 is suitable for representing 3D points or vectors
    tf2::Vector3 point_in_tf2(data_in.x, data_in.y, data_in.z);

    // 2. Convert geometry_msgs::Transform to tf2::Transform
    //    A tf2::Transform internally stores a rotation (Quaternion) and a translation (Vector3).
    //    It provides convenient methods for applying the transform (e.g., operator*).
    //    The standard transform application is P_out = R * P_in + T,
    //    where R is rotation, T is translation.
    tf2::Quaternion rotation_tf2(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    );

    tf2::Vector3 translation_tf2(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    );

    tf2::Transform transform_tf2(rotation_tf2, translation_tf2);

    // 3. Apply the transform to the point using tf2's operator*
    //    tf2's operator* for Transform * Vector3 correctly performs R * point + T
    tf2::Vector3 point_out_tf2 = transform_tf2 * point_in_tf2;

    // 4. Convert the resulting tf2::Vector3 back to geometry_msgs::Point
    data_out.x = point_out_tf2.x();
    data_out.y = point_out_tf2.y();
    data_out.z = point_out_tf2.z();
}

void doPoseTransform(
    const geometry_msgs::msg::Pose &data_in,
    geometry_msgs::msg::Pose &data_out,
    const geometry_msgs::msg::TransformStamped &transform)
{
    tf2::Transform transform_tf2;
    transform_tf2.setOrigin(tf2::Vector3(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    ));
    transform_tf2.setRotation(tf2::Quaternion(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    ));

    tf2::Vector3 position_in_tf2;
    position_in_tf2.setX(data_in.position.x);
    position_in_tf2.setY(data_in.position.y);
    position_in_tf2.setZ(data_in.position.z);


    tf2::Vector3 position_out_tf2 = transform_tf2 * position_in_tf2;

    data_out.position.x = position_out_tf2.x();
    data_out.position.y = position_out_tf2.y();
    data_out.position.z = position_out_tf2.z();

    tf2::Quaternion orientation_in_tf2;
    orientation_in_tf2.setX(data_in.orientation.x);
    orientation_in_tf2.setY(data_in.orientation.y);
    orientation_in_tf2.setZ(data_in.orientation.z);
    orientation_in_tf2.setW(data_in.orientation.w);

    tf2::Quaternion orientation_out_tf2 = transform_tf2.getRotation() * orientation_in_tf2;

    orientation_out_tf2.normalize();

    data_out.orientation.x = orientation_out_tf2.x();
    data_out.orientation.y = orientation_out_tf2.y();
    data_out.orientation.z = orientation_out_tf2.z();
    data_out.orientation.w = orientation_out_tf2.w();
}

}// Engineering_robot_Pnx