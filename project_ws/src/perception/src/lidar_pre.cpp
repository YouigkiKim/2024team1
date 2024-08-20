#include "lidar_pre.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/colors.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <derived_object_msgs/Object.h>
#include <derived_object_msgs/ObjectArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "matplotlibcpp.h"
#include "localization.h"
#include "KalmanFilter.h"
#include "perception/vehicle_state.h"
#include <custom_msgs/vehicle_state.h>

namespace plt = matplotlibcpp;

struct MyPointCloud {
    pcl::PointXYZRGB point;
    float angle;
    float distance;
    int cluster_id;
};

// 모두 transform2Carla를 통해서 시각화
struct MyCluster {
    int cluster_id;
    float center_point_x;
    float center_point_y;
    float center_point_z;
    std::vector<std::pair<float, float>> rectangle_points; // 4개의 꼭짓점 좌표 (x, y)
    float depth;
    float box_scale_x;
    float box_scale_y;
    float box_scale_z;
};


class LidarPreprocessor {
private:
    ros::NodeHandle nh_;
    ros::Publisher point_pub_;
    ros::Publisher cluster_pub_;  // 클러스터 퍼블리셔 추가
    ros::Publisher marker_pub_;   // marker array
    ros::Publisher object_array_pub_;
    ros::Subscriber point_sub_;
    ros::Subscriber state_sub_;
    pcl::PCLPointCloud2::Ptr cloud_out_PCL2_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    
    std::mutex cloud_mutex_;
    std::vector<double> dt_s;
    std::vector<double> number_of_point_s;
    std::vector<double> cluster_id_s;

    EgoState ego_;
    
    std::atomic<bool> stop_visualization_;
    // Reference path
    std::vector<float> reference_xs;
    std::vector<float> reference_ys;

    // Tracking
    std::vector<MyCluster> previous_clusters_;
    // Kalman 필터 객체를 관리하기 위한 맵 추가
    std::map<int, KalmanFilter2> kalman_filters_;  // 클러스터 ID에 따른 Kalman 필터 관리
    std::map<int, int> cluster_lifetime_;

    double last_time = 0.0;
    double dt = 0.0;


    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    void applyPassthroughFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output);
    void applyCarRegionFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output);
    void applyErrorFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output);
    void applyGroudnRemovalFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output);
    void applyVoxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input);
    void clustering_points(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input);
    void downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output, float radius, int max_points);
    // void transform2Carla(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, std::vector<MyPointCloud>& my_point_cloud);
    void transform2Carla_clusters(std::vector<MyCluster>& my_clusters);
    void transform2Carla_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud);

    void initializeReferencePath();
    void vehicleStateCallback(const custom_msgs::vehicle_state::ConstPtr& msg);
    // void visualize_2Dpoints(std::vector<MyPointCloud>& my_point_cloud);
    void visualize_2Dpoints(std::vector<MyCluster>& my_clusters);

    // 실험
    void processAndVisualizeBoundingBox(const std::vector<MyPointCloud>& my_point_cloud);
    void broadcastCarlaTransform(const EgoState& ego_state);

    void tracking(std::vector<MyCluster>& my_clusters, double dt);



    

public:
    LidarPreprocessor();
    void spin();
    void visualize();
    // static int count;
};

// int LidarPreprocessor::count = 0;

LidarPreprocessor::LidarPreprocessor()
    : cloud_out_(new pcl::PointCloud<pcl::PointXYZI>),
      cloud_out_PCL2_(new pcl::PCLPointCloud2), stop_visualization_(false)
{
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("voxel", 1);
    cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("clusters", 1);  // 클러스터 퍼블리셔 초기화
    // point_sub_ = nh_.subscribe("/carla/hero/front", 1, &LidarPreprocessor::cloudCallback, this);
    point_sub_ = nh_.subscribe("/carla/hero/LIDAR", 1, &LidarPreprocessor::cloudCallback, this);

    state_sub_ = nh_.subscribe("/carla/hero/pose", 1, &LidarPreprocessor::vehicleStateCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("bounding_box_marker", 1);
    object_array_pub_ = nh_.advertise<derived_object_msgs::ObjectArray>("object_data_array", 10);


    initializeReferencePath();
}

void LidarPreprocessor::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // KdTree 객체 생성 - 클러스터링을 위한 포인트 클라우드 탐색
    pcl::fromROSMsg(*input, *cloud);
    // ROS_INFO("call back count : %d" , count);

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        applyPassthroughFilter(cloud, cloud_out_);
        applyGroudnRemovalFilter(cloud_out_, cloud_out_);
        applyCarRegionFilter(cloud_out_, cloud_out_);
        applyErrorFilter(cloud_out_, cloud_out_);
        applyVoxelGridFilter(cloud_out_);

    }

    if (cloud_out_PCL2_) {
        sensor_msgs::PointCloud2 output;
        pcl_conversions::fromPCL(*cloud_out_PCL2_, output);
        output.header.frame_id = "hero/front";
        output.header.stamp = ros::Time::now();
        point_pub_.publish(output);

        clustering_points(cloud_out_);
    } else {
        ROS_ERROR("cloud_out_PCL2_ is null, cannot convert to sensor_msgs::PointCloud2");
    }
}

void LidarPreprocessor::applyPassthroughFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.1, 0.0);
    pass.filter(*output);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(-60, 60);
    pass.setInputCloud(output);
    pass.filter(*output);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(-15, 15);
    pass.setInputCloud(output);
    pass.filter(*output);
}

void LidarPreprocessor::applyCarRegionFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {
    pcl::CropBox<pcl::PointXYZI> box_filter;
    box_filter.setInputCloud(input);
    box_filter.setMin(Eigen::Vector4f(-2.4, -0.9, -std::numeric_limits<float>::max(), 1.0));
    box_filter.setMax(Eigen::Vector4f(2.2, 0.9, std::numeric_limits<float>::max(), 1.0));
    box_filter.setNegative(true);
    box_filter.filter(*output);
}

void LidarPreprocessor::applyErrorFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {
    // 첫 번째 영역을 필터링
    pcl::CropBox<pcl::PointXYZI> box_filter;
    box_filter.setInputCloud(input);

    // 잘라내고 싶은 영역의 min, max 좌표 설정
    box_filter.setMin(Eigen::Vector4f(-10.0, -2.0, -0.55, 1.0));
    box_filter.setMax(Eigen::Vector4f(4.0, 0.0, -0.0, 1.0));
    
    box_filter.setNegative(true);
    box_filter.filter(*output);
}


void LidarPreprocessor::applyGroudnRemovalFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_low(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_high(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints_neg(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.1, -1.0);
    pass.filter(*cloud_low);

    pass.setFilterLimits(-1.0, std::numeric_limits<float>::max());
    pass.filter(*cloud_high);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(20);
    seg.setDistanceThreshold(0.3);

    seg.setInputCloud(cloud_low);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud_low);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*inlierPoints_neg);

    *output = *inlierPoints_neg + *cloud_high;
}

void LidarPreprocessor::downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output, float radius = 5.0f, int max_points = 500) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr close_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr far_points(new pcl::PointCloud<pcl::PointXYZI>);

    // 5m 이내의 포인트와 그 외의 포인트를 분리
    for (const auto& point : input->points) {
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance <= radius) {
            close_points->points.push_back(point);
        } else {
            far_points->points.push_back(point);
        }
    }

    // 5m 이내의 포인트 중에서 개수를 500개로 제한하여 다운샘플링
    if (close_points->points.size() > max_points) {
        std::random_shuffle(close_points->points.begin(), close_points->points.end());
        close_points->points.resize(max_points);
    }

    close_points->width = close_points->points.size();
    close_points->height = 1;
    close_points->is_dense = true;

    // 다운샘플링된 포인트와 나머지 포인트를 결합
    *output = *close_points + *far_points;

    // ROS_INFO("Downsampled point cloud (within 5m): %d points", close_points->width * close_points->height);
    // ROS_INFO("Total point cloud after merging: %d points", output->width * output->height);
}

void LidarPreprocessor::applyVoxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) {
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2::Ptr cloud_out_PCL2(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*input, *cloud_out_PCL2);

    sor.setInputCloud(cloud_out_PCL2);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);
    sor.filter(*cloud_out_PCL2_);

    
    pcl::fromPCLPointCloud2(*cloud_out_PCL2_, *input);

    // ROS_INFO("Voxelized point cloud 개수: %d", cloud_out_PCL2_->width * cloud_out_PCL2_->height);
    
}


void LidarPreprocessor::processAndVisualizeBoundingBox(const std::vector<MyPointCloud>& my_point_cloud) {
    std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cluster_map;
    std::vector<MyCluster> my_cluster;

    // 클러스터 ID별로 포인트 클라우드 그룹핑
    for (const auto& point : my_point_cloud) {
        if (cluster_map.find(point.cluster_id) == cluster_map.end()) {
            cluster_map[point.cluster_id] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        }
        cluster_map[point.cluster_id]->points.push_back(point.point);
    }

    for (const auto& cluster : cluster_map) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud = cluster.second;

        // Convex Hull 계산
        pcl::ConvexHull<pcl::PointXYZRGB> hull;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
        hull.setInputCloud(cluster_cloud);
        hull.reconstruct(*cloud_hull);

        if (cloud_hull->points.size() < 3) {
            continue; // 유효한 Convex Hull이 아닌 경우 스킵
        }

        // Min-Max 3D points 계산
        pcl::PointXYZRGB min_point, max_point;
        pcl::getMinMax3D(*cloud_hull, min_point, max_point);

        // Min-Max 포인트 출력
        // ROS_INFO("Min Point: (%.5f, %.5f, %.5f)", min_point.x, min_point.y, min_point.z);
        // ROS_INFO("Max Point: (%.5f, %.5f, %.5f)", max_point.x, max_point.y, max_point.z);
        
        float bbox_width = abs(max_point.x - min_point.x);
        float bbox_height = abs(max_point.y - min_point.y);
        float bbox_depth = abs(max_point.z - min_point.z);
        // ROS_INFO("BOX SCALE X: %.5f  Y: %.5f  Z: %.5f", bbox_width, bbox_height, bbox_depth);
        if (bbox_width > 6.0 || bbox_height > 6.0) {
            // ROS_WARN("Bounding Box size exceeds 6 meters, skipping this box.");
            continue;
        }

        float center_x = (min_point.x + max_point.x) / 2.0;
        float center_y = (min_point.y + max_point.y) / 2.0;
        float center_z = (min_point.z + max_point.z) / 2.0;

        std::vector<std::pair<float, float>> rectangle_points = {
            {min_point.x, min_point.y},
            {max_point.x, min_point.y},
            {max_point.x, max_point.y},
            {min_point.x, max_point.y}
        };

        MyCluster cluster_data;
        cluster_data.cluster_id = cluster.first;
        cluster_data.center_point_x = center_x;
        cluster_data.center_point_y = center_y;
        cluster_data.center_point_z = center_z;
        cluster_data.depth = bbox_depth;
        cluster_data.rectangle_points = rectangle_points;
        cluster_data.box_scale_x = bbox_width;
        cluster_data.box_scale_y = bbox_height;
        cluster_data.box_scale_z = bbox_depth;
        my_cluster.push_back(cluster_data);
    }

    // Transform the cluster data to Carla coordinate system for further use
    transform2Carla_clusters(my_cluster);
    
    double current_time= ros::Time::now().toSec();
    dt = current_time - last_time;
    if(dt > 1){
        dt = 0.0;
    }
    last_time = current_time;
    ROS_INFO("dt : %.5f" , dt);
    tracking(my_cluster, dt);

    

    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;

    for (const auto& cluster : my_cluster) {
        auto& kalman_filter = kalman_filters_[cluster.cluster_id];
        Eigen::Vector4d predicted_state = kalman_filter.getState();
        double predicted_x = predicted_state[0];
        double predicted_y = predicted_state[1];
        double predicted_vx = predicted_state[2];
        double predicted_vy = predicted_state[3];

        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = "map";  // Carla 좌표계로 변경
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "bounding_box";
        bbox_marker.id = marker_id++;
        bbox_marker.type = visualization_msgs::Marker::CUBE;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        bbox_marker.pose.position.x = cluster.center_point_x;
        bbox_marker.pose.position.y = cluster.center_point_y;
        bbox_marker.pose.position.z = cluster.center_point_z;
        // 회전 설정 (ego.heading을 쿼터니언으로 변환하여 적용)
        tf2::Quaternion q;
        q.setRPY(0, 0, ego_.heading);  // Roll, Pitch는 0, Yaw는 ego.heading 사용
        bbox_marker.pose.orientation.x = q.x();
        bbox_marker.pose.orientation.y = q.y();
        bbox_marker.pose.orientation.z = q.z();
        bbox_marker.pose.orientation.w = q.w();
        bbox_marker.scale.x = cluster.box_scale_x;
        bbox_marker.scale.y = cluster.box_scale_y;
        bbox_marker.scale.z = cluster.box_scale_z;
        bbox_marker.color.r = 0.0;
        bbox_marker.color.g = 1.0;
        bbox_marker.color.b = 0.0;
        bbox_marker.color.a = 0.5;
        bbox_marker.lifetime = ros::Duration(0.1);

        marker_array.markers.push_back(bbox_marker);

        std::stringstream ss;
        ss << "ID: " << cluster.cluster_id << "\n"
           << "x: " << predicted_x << "\n"
           << "y: " << predicted_y ;
        //    << "vx: " << predicted_vx << "\n"
        //    << "vy: " << predicted_vy;

        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";  // Carla 좌표계로 변경
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "predicted_state";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = cluster.center_point_x;
        text_marker.pose.position.y = cluster.center_point_y;
        text_marker.pose.position.z = cluster.center_point_z + bbox_marker.scale.z / 2.0 + 0.5;
        text_marker.pose.orientation.x = q.x();
        text_marker.pose.orientation.y = q.y();
        text_marker.pose.orientation.z = q.z();
        text_marker.pose.orientation.w = q.w();
        text_marker.scale.z = 0.5;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = ss.str();
        text_marker.lifetime = ros::Duration(0.1);

        marker_array.markers.push_back(text_marker);
    }

    marker_pub_.publish(marker_array);

    derived_object_msgs::ObjectArray object_array_msg;
    object_array_msg.header.stamp = ros::Time::now();
    object_array_msg.header.frame_id = "map";  // Carla 좌표계로 변경

    for (const auto& cluster : my_cluster) {
        derived_object_msgs::Object object_msg;
        object_msg.id = cluster.cluster_id;
        object_msg.detection_level = derived_object_msgs::Object::OBJECT_DETECTED;

        object_msg.pose.position.x = cluster.center_point_x;
        object_msg.pose.position.y = cluster.center_point_y;
        object_msg.pose.position.z = cluster.center_point_z;

        object_msg.shape.type = shape_msgs::SolidPrimitive::BOX;
        object_msg.shape.dimensions.resize(3);
        object_msg.shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = cluster.box_scale_x;
        object_msg.shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = cluster.box_scale_y;
        object_msg.shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = cluster.box_scale_z;

        for (const auto& point : cluster.rectangle_points) {
            geometry_msgs::Point32 polygon_point;
            polygon_point.x = point.first;
            polygon_point.y = point.second;
            polygon_point.z = 0.0;
            object_msg.polygon.points.push_back(polygon_point);
        }

        object_array_msg.objects.push_back(object_msg);
    }

    object_array_pub_.publish(object_array_msg);
}


void LidarPreprocessor::tracking(std::vector<MyCluster>& my_clusters, double dt) {

    // 1.  첫 스텝에서는 previous_clusters가 없으니 kalmanfilter 초기화하고 다음 스텝까지 대기

    if (previous_clusters_.empty()) {
        // 이전 클러스터가 없는 경우 Kalman 필터 초기화
        for (auto& cluster : my_clusters) {
            KalmanFilter2 kf;
            Eigen::Vector4d initial_state(cluster.center_point_x, cluster.center_point_y, 0.0, 0.0); // 초기 속도는 0으로 가정
            kf.init(initial_state);
            kalman_filters_[cluster.cluster_id] = kf;
        }
        previous_clusters_ = my_clusters;
        return;
    }

    std::vector<MyCluster> updated_clusters;

    // 2. 2 번째 스텝부터 kalman filter를 이용한 predict -> data association -> update 시작


    for (auto& new_cluster : my_clusters) {
        double min_distance = std::numeric_limits<double>::max();
        int matched_id = -1;

        // 3. Kalman Filter predict 단계
        for (auto& previous_cluster : previous_clusters_) {
            auto& kalman_filter = kalman_filters_[previous_cluster.cluster_id];
            kalman_filter.predict(dt);

            // 4. Data association
            // 예측된 위치로부터 거리 계산
            Eigen::Vector4d predicted_position = kalman_filter.getState();
            double distance = std::sqrt(
                std::pow(new_cluster.center_point_x - predicted_position[0], 2) +
                std::pow(new_cluster.center_point_y - predicted_position[1], 2)
            );

            if (distance < min_distance) {
                min_distance = distance;
                matched_id = previous_cluster.cluster_id;
            }
        }

        // 제일 가까운 cluster_id를 찾았으니 min_distance와 비교 min_distance가 2.0보다 크면 새로운 cluster_id 부여
        if (min_distance < 3.0) {
            // ROS_INFO("Cluster ID %d matched with previous cluster ID %d. Distance: %.2f", new_cluster.cluster_id, matched_id, min_distance);
            new_cluster.cluster_id = matched_id;

            // 측정값을 사용하여 Kalman 필터 업데이트
            auto& kalman_filter = kalman_filters_[new_cluster.cluster_id];
            Eigen::Vector2d measurement(new_cluster.center_point_x, new_cluster.center_point_y);

            // 5. Update 단계
            kalman_filter.update(measurement);

            
        } else {
            new_cluster.cluster_id = previous_clusters_.size() + updated_clusters.size() + 1;
            KalmanFilter2 new_kf;
            Eigen::Vector4d initial_state(new_cluster.center_point_x, new_cluster.center_point_y, 0.0, 0.0);
            new_kf.init(initial_state);
            kalman_filters_[new_cluster.cluster_id] = new_kf;

            // ROS_INFO("New cluster detected. Assigned new ID %d", new_cluster.cluster_id);
        }


        updated_clusters.push_back(new_cluster);
    }

    previous_clusters_ = updated_clusters;
}



void LidarPreprocessor::clustering_points(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) {
    std::vector<MyPointCloud> my_point_cloud;
    if (input->empty()) {
        ROS_WARN("Input point cloud is empty. Skipping clustering.");
        return;
    }
    
    
    
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(input);

    // 클러스터 인덱스를 저장할 벡터
    std::vector<pcl::PointIndices> cluster_indices;

    // Euclidean 클러스터링 객체 생성
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.85); // 클러스터링 거리 임계값 (단위: meter)
    ec.setMinClusterSize(5);   // 클러스터의 최소 포인트 개수
    ec.setMaxClusterSize(25000); // 클러스터의 최대 포인트 개수
    ec.setSearchMethod(tree);    // 탐색 방법으로 KdTree 사용
    ec.setInputCloud(input);
    ec.extract(cluster_indices); // 클러스터링 실행
    
    // // 총 클러스터 ID 수 visualize를 위해 저장
    // cluster_id_s.push_back(cluster_indices.size());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 각 클러스터를 개별적으로 처리
    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {
        // 무작위 색상 생성
        uint8_t r = static_cast<uint8_t>(rand() % 256);
        uint8_t g = static_cast<uint8_t>(rand() % 256);
        uint8_t b = static_cast<uint8_t>(rand() % 256);

        for (const auto& index : indices.indices) {
            pcl::PointXYZRGB point;
            point.x = input->points[index].x;
            point.y = input->points[index].y;
            point.z = input->points[index].z;
            point.r = r;
            point.g = g;
            point.b = b;
            colored_cloud->points.push_back(point);

            // Populate MyPointCloud with cluster_id
            MyPointCloud my_point;
            my_point.point = point;
            my_point.angle = std::atan2(point.y, point.x) * 180 / M_PI;
            my_point.distance = std::sqrt(point.x * point.x + point.y * point.y);
            my_point.cluster_id = cluster_id; // Set the cluster_id
            my_point_cloud.push_back(my_point);
        }

        ROS_INFO("Cluster %d: %zu points", cluster_id++, indices.indices.size());
    }

    colored_cloud->width = colored_cloud->points.size();
    colored_cloud->height = 1;
    colored_cloud->is_dense = true;

    transform2Carla_points(colored_cloud);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();
    cluster_pub_.publish(output);
    
    // number_of_point_s.push_back(cloud_out_->width * cloud_out_->height);

    // 각 클러스터에 대해 Bounding Box 생성 및 시각화
    processAndVisualizeBoundingBox(my_point_cloud);
    
    
}
void LidarPreprocessor::transform2Carla_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud) {
    Eigen::Matrix3f R;
    R << std::cos(-ego_.heading), std::sin(-ego_.heading), ego_.x,
         -std::sin(-ego_.heading), std::cos(-ego_.heading), ego_.y,
         0, 0, 1;

    for (auto& point : colored_cloud->points) {
        Eigen::Vector3f original_point(point.x, point.y, 1.0f);
        Eigen::Vector3f transformed_point = R * original_point;

        point.x = transformed_point[0];
        point.y = transformed_point[1];
        // Z값은 변하지 않음
    }
}
void LidarPreprocessor::transform2Carla_clusters(std::vector<MyCluster>& my_clusters) {
    // 변환 행렬 정의 (왼손 좌표계를 고려하여 y축 반전)
    Eigen::Matrix3f R;
    R << std::cos(-ego_.heading), std::sin(-ego_.heading), ego_.x,
         -std::sin(-ego_.heading), std::cos(-ego_.heading), ego_.y,
         0, 0, 1;

    for (auto& cluster : my_clusters) {
        // 중심점 변환
        Eigen::Vector3f center_point(cluster.center_point_x, cluster.center_point_y, 1.0f);
        Eigen::Vector3f center_point_carla = R * center_point;

        // 변환된 중심점 업데이트
        cluster.center_point_x = center_point_carla[0];
        cluster.center_point_y = center_point_carla[1];

        // 직육면체 바닥의 각 꼭짓점 변환
        for (auto& point : cluster.rectangle_points) {
            Eigen::Vector3f rect_point(point.first, point.second, 1.0f);
            Eigen::Vector3f rect_point_carla = R * rect_point;

            // 변환된 좌표로 업데이트
            point.first = rect_point_carla[0];
            point.second = rect_point_carla[1];
        }
    }
}


void LidarPreprocessor::visualize_2Dpoints(std::vector<MyCluster>& my_clusters) {
    plt::clf();
    std::vector<std::string> colors = {"r", "g", "b", "c", "m", "y", "k"};

    int color_idx = 0;
    for (const auto& cluster : my_clusters) {
        std::vector<double> x_vals, y_vals;

        // 각 클러스터의 직육면체 바닥의 꼭짓점들을 좌표 리스트에 추가
        for (const auto& rect_point : cluster.rectangle_points) {
            x_vals.push_back(rect_point.first);
            y_vals.push_back(rect_point.second);
        }

        // 클러스터의 바닥 직사각형을 닫기 위해 첫 번째 점을 다시 추가
        if (!x_vals.empty() && !y_vals.empty()) {
            x_vals.push_back(x_vals.front());
            y_vals.push_back(y_vals.front());
        }

        // 클러스터를 색깔별로 표시
        plt::scatter(x_vals, y_vals, 10.0, {{"color", colors[color_idx % colors.size()]}});
        plt::plot(x_vals, y_vals, {{"color", colors[color_idx % colors.size()]}});
        
        color_idx++;
    }

    double range = 15.0;
    plt::plot(reference_xs, reference_ys, "g-");
    plt::plot({ego_.x}, {ego_.y}, "ro");
    plt::grid(true);
    plt::title("Clusters in Carla 2D");
    plt::xlabel("Carla X");
    plt::ylabel("Carla Y");
    plt::xlim(ego_.x - range, ego_.x + range);
    plt::ylim(ego_.y - range, ego_.y + range);
    plt::pause(0.01);
}

void LidarPreprocessor::broadcastCarlaTransform(const EgoState& ego) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "hero";

    transformStamped.transform.translation.x = ego.x;
    transformStamped.transform.translation.y = ego.y;
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, ego.heading); // Roll, Pitch, Yaw 설정
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // hero/front 프레임의 변환 설정
    geometry_msgs::TransformStamped front_transform;
    front_transform.header.stamp = ros::Time::now();
    front_transform.header.frame_id = "hero";
    front_transform.child_frame_id = "hero/front";

    front_transform.transform.translation.x = 0.0;
    front_transform.transform.translation.y = 0.0;
    front_transform.transform.translation.z = 2.0; // z 축에서 2.0만큼 올라감

    // hero와 같은 방향을 유지하기 위해 회전은 0으로 설정
    front_transform.transform.rotation.x = 0.0;
    front_transform.transform.rotation.y = 0.0;
    front_transform.transform.rotation.z = 0.0;
    front_transform.transform.rotation.w = 1.0;

    // Transform을 브로드캐스트
    br.sendTransform(front_transform);
}

void LidarPreprocessor::visualize(){
    
    plt::figure();
    // plt::subplot(3,1,1);
    plt::plot(dt_s , "b-");
    plt::grid(true);
    plt::xlabel("Number of Clustering iterations");
    plt::ylabel("dt [sec]");
    plt::title(" clustering time ");
    // plt::axis("equal");

    // plt::subplot(,1,2);
    plt::figure();
    plt::plot(number_of_point_s , "r-");
    plt::grid(true);
    plt::xlabel("Number of Clustering iterations");
    plt::ylabel("Number of Pointcloud");
    plt::title(" Number of Pointcloud ");
    // plt::axis("equal");


    // plt::subplot(3,1,3);
    plt::figure();
    plt::plot(cluster_id_s , "g-");
    plt::grid(true);
    plt::xlabel("Number of Clustering iterations");
    plt::ylabel("Number of Cluste_id");
    plt::title(" Number of Cluster_id ");

    // plt::show();

    


}

void LidarPreprocessor::initializeReferencePath() {
    reference_xs = {
        6774.62,    6770.899414, 6770.933594, 6770.975586, 6771.009766, 6771.043457, 6771.085938,
        6771.120117, 6771.153809, 6771.187988, 6771.221680, 6771.255859, 6771.289551,
        6771.332031, 6771.366211, 6771.399902, 6771.442383, 6771.476562, 6771.510254,
        6771.552734, 6771.586914, 6771.620605, 6771.663086, 6771.697266, 6771.730957,
        6771.773438, 6771.807617, 6771.841309, 6771.875488, 6771.909180, 6771.943359,
        6771.977051, 6772.019531, 6772.053711, 6772.087402, 6772.129883, 6772.163574,
        6772.197754, 6772.240234, 6772.273926, 6772.308105, 6772.341797, 6772.375977,
        6772.409668, 6772.443848, 6772.486328, 6772.520020
    };
    reference_ys = {
        5440.85,     5426.435547, 5422.435547, 5417.436035, 5413.436035, 5409.436035, 5404.436523,
        5400.436523, 5396.436523, 5392.437012, 5388.437012, 5384.437012, 5380.437012,
        5375.437500, 5371.437500, 5367.437500, 5362.437988, 5358.437988, 5354.437988,
        5349.438477, 5345.438477, 5341.438477, 5336.438965, 5332.438965, 5328.438965,
        5323.439453, 5319.439453, 5315.439453, 5311.439941, 5307.439941, 5303.439941,
        5299.439941, 5294.440430, 5290.440430, 5286.440430, 5281.440918, 5277.440918,
        5273.440918, 5268.441406, 5264.441406, 5260.441406, 5256.441895, 5252.441895,
        5248.441895, 5244.441895, 5239.442383, 5235.442383
    };
}
void LidarPreprocessor::vehicleStateCallback(const custom_msgs::vehicle_state::ConstPtr& msg) {
    ego_.x = msg->x;
    ego_.y = msg->y;
    ego_.heading = msg->heading;
}

void LidarPreprocessor::spin() {
    plt::ion(); // Enable interactive mode
    ros::Rate rate(20);
    std::thread([this, &rate]() {
        while (ros::ok() && !stop_visualization_) {
            {
                // std::lock_guard<std::mutex> lock(cloud_mutex_);
                ros::spinOnce();
                broadcastCarlaTransform(ego_);
                clustering_points(cloud_out_);
                rate.sleep();
                // visualize();
            }  
        }
    }).detach();;

    std::cout << "Press Enter to stop visualization..." << std::endl;
    std::cin.get(); // Wait for Enter key press
    stop_visualization_ = true;

    plt::close(); // Close the plot window

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_pre_node");

    LidarPreprocessor preprocessor;

    preprocessor.spin();
    // preprocessor.visualize();


    return 0;
}
