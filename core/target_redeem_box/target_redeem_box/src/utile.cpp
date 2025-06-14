#include "target_redeem_box/utile.hpp"

const double eps = 1e-8;

template<typename T,typename G>
double distance_points(const cv::Point_<T> & p1,const cv::Point_<G> & p2){
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

template double distance_points<int,int> (const cv::Point_<int> & p1,const cv::Point_<int> & p2);
template double distance_points<float,float> (const cv::Point_<float> & p1,const cv::Point_<float> & p2);
template double distance_points<double,double> (const cv::Point_<double> & p1,const cv::Point_<double> & p2);
template double distance_points<int,double> (const cv::Point_<int> & p1,const cv::Point_<double> & p2);
template double distance_points<double,int> (const cv::Point_<double> & p1,const cv::Point_<int> & p2);
template double distance_points<float,int> (const cv::Point_<float> & p1,const cv::Point_<int> & p2);
template double distance_points<int,float> (const cv::Point_<int> & p1,const cv::Point_<float> & p2);
template double distance_points<float,double> (const cv::Point_<float> & p1,const cv::Point_<double> & p2);
template double distance_points<double,float> (const cv::Point_<double> & p1,const cv::Point_<float> & p2);

double angle_according_to_horizon(cv::Point p1,cv::Point p2){
    cv::Point horison(1,0),tar=p1-p2;
    if(tar.y>=eps) tar=-tar;
    double dot_=tar.dot(horison);
    double cos_=dot_/(cv::norm(tar)*cv::norm(horison));
    double angle=std::acos(cos_)/CV_PI*180;
    return angle;
}

bool pnp_solver(const std::vector<cv::Point2f > & image_points2D,
    const std::vector<cv::Point3f > & object_points3D,
    const std::vector<double> & camera_matrix,
    const std::vector<double> & dist_coeffs,
    cv::Mat & rvec, 
    cv::Mat & tvec, 
    bool useExtrinsicGuess, 
    int flags){

    cv::Mat camera_matrixCV=cv::Mat(3,3,CV_64F,const_cast<double *>(camera_matrix.data())).clone();
    cv::Mat dist_coeffsCV=cv::Mat(1,5,CV_64F,const_cast<double *>(dist_coeffs.data())).clone();

    bool pnp_success=0;
    
    try{
        pnp_success=cv::solvePnP(object_points3D,
            image_points2D,
            camera_matrixCV,
            dist_coeffsCV,
            rvec,
            tvec,
            useExtrinsicGuess,
            flags);
    }
    catch(std::exception & e){
        RCLCPP_ERROR(rclcpp::get_logger("pnp_solver"),"error: %s",e.what());
        return false;
    }

    if(!pnp_success){
        return false;
    }

    for(int i=0;i<3;i++){
        if(std::isnan(rvec.at<double>(i))||std::isnan(tvec.at<double>(i))){
            RCLCPP_ERROR(rclcpp::get_logger("pnp_solver"),"PNP get NAN!!!!!");
            return false;
        }
    }

    return true;
}

template<typename T>
void draw_pnp_result(cv::Mat image, 
    const cv::Mat & rvec,
    const cv::Mat & tvec,
    const std::vector<double> & camera_matrix,
    const std::vector<cv::Point3_<T>> object_points,
    cv::Scalar color, 
    int thickness, 
    cv::Point textpos,
    bool draw_xyz){

    if(!(((rvec.rows==3&&rvec.cols==1)||(rvec.rows==1&&rvec.cols==3))&&
    ((tvec.rows==3&&tvec.cols==1)||(tvec.rows==1&&tvec.cols==3)))){
        RCLCPP_ERROR(rclcpp::get_logger("draw_pnp_result"),"rvec or tvec is not 3*1 or 1*3");
        return;
    }

    for(int i=0;i<3;i++){
        if(std::isnan(rvec.at<double>(i))||std::isnan(tvec.at<double>(i))){
            RCLCPP_ERROR(rclcpp::get_logger("draw_pnp_result"),"rvec or tvec have nan");
            return;
        }
    }

    // 旋转矩阵
    cv::Mat rmat;
    // 齐次化平移矩阵
    Eigen::Matrix<double,4,4> rtvec_eigen;
    // 相机内参矩阵
    Eigen::Matrix<double,3,3> camera_matrix_eigen;
    camera_matrix_eigen<<camera_matrix[0],0,camera_matrix[2],
        0,camera_matrix[1],camera_matrix[3],
        0,0,1;

    // object_points 的 eigen 版本
    std::vector<Eigen::Matrix<double,4,1>> object_points_eigen;
    for(const auto & i : object_points){
        object_points_eigen.push_back(Eigen::Matrix<double,3,1>(i.x,i.y,i.z,1));
    }

    // 降维矩阵
    Eigen::Matrix<double,3,4> sign_mat;
    sign_mat<<1,0,0,0,
        0,1,0,0,
        0,0,1,0;
    
    // 投影结果
    std::vector<cv::Point2i> result_points;

    // 原来的 原点和xyz 轴
    std::vector<Eigen::Matrix<double,4,1>>  xyz_axis_eigen={
        Eigen::Matrix<double,4,1>(0,0,0,1),
        Eigen::Matrix<double,4,1>(1,0,0,1),
        Eigen::Matrix<double,4,1>(0,1,0,1),
        Eigen::Matrix<double,4,1>(0,0,1,1)
    };

    // 其次化转化过后的xyz轴
    std::vector<Eigen::Matrix<double,3,1>>  image_xyz_axis_eigen;

    cv::Rodrigues(rvec,rmat);

    for(int i=0;i<3;i++){
        for(int e=0;e<3;e++){
            rtvec_eigen(i,e)=rmat.at<double>(i,e);
        }
        rtvec_eigen(i,3)=tvec.at<double>(i);
    }
    for(int i=0;i<3;i++) rtvec_eigen(3, i) = 0.0;
    rtvec_eigen(3, 3) = 1.0;

    for(const auto & i : object_points_eigen){
        Eigen::Matrix<double,3,1> coordination=camera_matrix_eigen*sign_mat*rtvec_eigen*i;
        coordination/=coordination(2);
        result_points.push_back(cv::Point2i(coordination(0),coordination(1)));
    }

    for(const auto & i : xyz_axis_eigen){
        Eigen::Matrix<double,3,1> coordination=camera_matrix_eigen*sign_mat*rtvec_eigen*i;
        coordination/=coordination(2);
        image_xyz_axis_eigen.push_back(cv::Point2i(coordination(0),coordination(1)));
    }

    cv::drawContours(image,
        std::vector<std::vector<cv::Point2i>>(1,result_points),
        -1,
        color,
        thickness);

    std::stringstream rvecss;
    std::stringstream tvecss; 
    rvecss<<"rvec: "<<rvec.at<double>(0)/CV_PI*180<<" "<<rvec.at<double>(1)/CV_PI*180<<" "<<rvec.at<double>(2)/CV_PI*180<<std::endl;
    tvecss<<"tvec: "<<tvec;

    cv::putText(image,rvecss.str().c_str(),textpos,cv::FONT_HERSHEY_SIMPLEX,1.0,color);
    cv::putText(image,tvecss.str().c_str(),cv::Point(textpos.x,textpos.y+25),cv::FONT_HERSHEY_SIMPLEX,1.0,color);

    if(draw_xyz){
        cv::line(image,image_xyz_axis_eigen[0],image_xyz_axis_eigen[1],color,thickness);
        cv::line(image,image_xyz_axis_eigen[0],image_xyz_axis_eigen[2],color,thickness);
        cv::line(image,image_xyz_axis_eigen[0],image_xyz_axis_eigen[3],color,thickness);
        cv::putText(image,"x",image_xyz_axis_eigen[1],cv::FONT_HERSHEY_SIMPLEX,1.0,color);
        cv::putText(image,"y",image_xyz_axis_eigen[2],cv::FONT_HERSHEY_SIMPLEX,1.0,color);
        cv::putText(image,"z",image_xyz_axis_eigen[3],cv::FONT_HERSHEY_SIMPLEX,1.0,color);
    }

}

template void draw_pnp_result<double>(cv::Mat image, 
    const cv::Mat & rvec, 
    const cv::Mat & tvec, 
    const std::vector<double> & camera_matrix,
    const std::vector<cv::Point3_<double>> object_points,
    cv::Scalar color, 
    int thickness, 
    cv::Point textpos,
    bool draw_xyz);

template void draw_pnp_result<float>(cv::Mat image, 
    const cv::Mat & rvec, 
    const cv::Mat & tvec, 
    const std::vector<double> & camera_matrix,
    const std::vector<cv::Point3_<float>> object_points,
    cv::Scalar color, 
    int thickness, 
    cv::Point textpos,
    bool draw_xyz);

/**
 * @brief find_polygon_counter_points_sets 的辅助函数，检测p是不是在 peaks 的忽略范围内
 * @param p 需要探测的点
 * @param peaks 顶点
 * @param PeaksThreshold 忽略范围
 * @return 如果有，则返回 pair<ture,对应的顶点>
 * @return 反之，则返回 std::make_pair(0,cv::Point(-1,-1))
 */
std::pair<bool,cv::Point> is_in_peak_threshold(const cv::Point & p,const std::vector<cv::Point> & peaks,const double PeaksThreshold){
    for(const auto & i : peaks){
        if(distance_points(p,i)<=PeaksThreshold) return std::make_pair(1,i);
    }
    return std::make_pair(0,cv::Point(-1,-1));
}

void find_polygon_counter_points_sets(const cv::Mat & binary_image,
    const std::vector<cv::Point> & peaks,
    const double peaks_ignore_radius,
    std::vector<std::vector<cv::Point>> & points_sets,
    std::vector<std::pair<cv::Point,cv::Point> >& end_points){

    // x上的位移
    static int dx[8]={0,0,1,-1,1,-1,1,-1};
    // y上的位移
    static int dy[8]={1,-1,0,0,1,1,-1,-1};

    int maxy=binary_image.rows;
    int maxx=binary_image.cols;

    // 用于记录已经被访问过的点
    cv::Mat vis=binary_image.clone();

    // 缩小扫描区域
    cv::Rect AOI=cv::boundingRect(binary_image);
    for(int i=AOI.x;i<AOI.x+AOI.width;i++){
        for(int e=AOI.y;e<AOI.y+AOI.height;e++){

            cv::Point now_point(i,e);

            if(vis.at<uchar>(e,i)==0) continue;
            if(is_in_peak_threshold(now_point,peaks,peaks_ignore_radius).first) continue;

            vis.at<uchar>(e,i)=1;

            // 当前的点集合
            std::vector<cv::Point> points_set;
            // 当前边的顶点
            std::pair<cv::Point,cv::Point> endpoints=std::make_pair(cv::Point(-1,-1),cv::Point(-1,-1)) ;

            std::queue<cv::Point> points_queue;

            // 找边
            while(!points_queue.empty()){
                cv::Point now_point=points_queue.front();
                points_queue.pop();
                for(int i=0;i<8;i++){
                    int nx=now_point.x+dx[i],ny=now_point.y+dy[i];
                    cv::Point next_point(nx,ny);

                    if(nx<0||nx>=maxx||ny<0||ny>=maxy) continue;
                    
                    if(vis.at<uchar>(ny,nx)==0) continue;

                    std::pair<bool,cv::Point> result=is_in_peak_threshold(next_point,peaks,peaks_ignore_radius);
                    // 添加端点
                    if(result.first){
                        if(endpoints.first==result.second||endpoints.second==result.second){
                            continue;
                        }
                        if(endpoints.first.x==-1){
                            endpoints.first=result.second;
                        }
                        else if(endpoints.second.x==-1){
                            endpoints.second=result.second;
                        }
                        continue;
                    }

                    points_set.push_back(next_point);
                    points_queue.push(next_point);
                    vis.at<uchar>(ny,nx)=0;
                }
            }

            points_sets.push_back(std::move(points_set));
            end_points.push_back(std::move(endpoints));
        }
    }

}