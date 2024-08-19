#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <perception/vehicle_state.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_msgs/CarlaSpeedometer.h>
#include <custom_msgs/vehicle_state.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <chrono>
#include <vector>
#include <cmath>
#include <random>
#include <queue>
#include "KalmanFilter.h"
#include "matplotlibcpp.h"
//tfpub
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>



namespace plt = matplotlibcpp;
using namespace Eigen;

struct EgoState {
    float x;
    float y;
    float heading;
};

Vector3d gnss_to_carla(const double& lat, const double& lon, const double& alt) {
    const double EARTH_RADIUS_EQUA = 6378137.0;  // WGS84 반장축 길이
    
    double lon_rad = lon * M_PI / 180.0;
    double lat_rad = std::log(std::tan((90.0 + lat) * M_PI / 360.0));
    double x = lon_rad * EARTH_RADIUS_EQUA;
    double y = -(lat_rad * EARTH_RADIUS_EQUA);
    return Vector3d(x, y, alt);
}

class Localization {
public:
    Localization() : kalman_filter(), received_gps(false), received_imu(false), generator(std::chrono::system_clock::now().time_since_epoch().count())
                    , distribution(0.0, 0.4) {
        ros::NodeHandle nh;

        // GNSS 데이터 구독
        gps_sub = nh.subscribe("/carla/hero/GPS", 1, &Localization::cb_gps, this);
        // gps_sub = nh.subscribe("/carla/hero/GPS", 1, &Localization::cb_gps, this);
        // gps_sub = nh.subscribe("/carla/ego_vehicle/gnss", 1, &Localization::cb_gps, this);

        // vehicle_status 데이터 구독
        // vehicle_status_sub = nh.subscribe("/carla/hero/vehicle_status", 1, &Localization::cb_vehicle_status, this);
        vehicle_status_sub = nh.subscribe("/carla/hero/IMU", 1, &Localization::cb_imu, this);
        // vehicle_status_sub = nh.subscribe("/carla/ego_vehicle/imu", 1, &Localization::cb_imu, this);

        // velocity 구독
        velocity_sub = nh.subscribe("/carla/hero/Speed", 1, &Localization::cb_velocity, this);
        // velocity_sub = nh.subscribe("/carla/ego_vehicle/speedometer", 1, &Localization::cb_velocity, this);


        // EKF 초기화
        kalman_filter.init(Eigen::Vector3d(6770.7749, 5441.1357, (-89.5/180*M_PI)));
        // kalman_filter.init(Eigen::Vector3d(-4338.97, 4328.48, 0.00967));

        // vehicle_state 데이터 발행
        state_pub = nh.advertise<custom_msgs::vehicle_state>("/carla/hero/pose", 1);

        // reference_path 설정
        initializeReferencePath();
    }
        void plotResults() {
        // plt::clf(); // 이전 그림 지우기
        plt::figure();
        plt::subplot(1,2,1);
        plt::plot(original_xs, original_ys, "k-"); // GNSS 좌표 점으로 표시
        plt::scatter(noisy_xs, noisy_ys, 4.0);
        plt::plot(filtered_xs, filtered_ys, "b-"); // 보정된 GPS 경로 파란색 선
        // plt::plot(reference_xs, reference_ys, "g-"); // 참조 경로 녹색 선

        plt::grid(true);

        plt::xlabel("X Axis [m]");
        plt::ylabel("Y Axis [m]");
        plt::title("Real-time GPS Visualization");

        // 등간격 비율 설정
        plt::axis("equal");

        plt::subplot(1,2,2);
        plt::plot(rmse_original, "r-");
        plt::plot(rmse_filtered, "b-");
        plt::grid(true);
        plt::xlabel("Number of updates");
        plt::ylabel("RMSE [m]");
        plt::title("RMSE (Noise GPS vs EKF Filtered GPS)");
        // plt::axis("equal");


        plt::figure();
        plt::subplot(2,1,1);
        plt::plot(velocity_s, "r-");
        plt::grid(true);
        plt::xlabel("Number of Speedometer Callback");
        plt::ylabel("velocity [m/s]");
        plt::title("velocity");

        plt::subplot(2,1,2);
        plt::plot(yaw_rate_s, "b-");
        plt::grid(true);
        plt::xlabel("Number of IMU Callback");
        plt::ylabel("yaw_rate [radian]");
        plt::title("yaw_rate");


     



        // plt::figure();
        // plt::subplot(3,1,1);
        // plt::plot(updated_P0, "g-");
        // plt::title("Pxx , Variance of X");
        // plt::grid(true);
        // plt::xlabel("Number of (prediction + updates)");
        // plt::ylabel("Variance[m^2]");
        
        // // plt::figure();
        // plt::subplot(3,1,2);
        // plt::plot(updated_P1, "k-");
        // plt::title("Pyy , Variance of Y");
        // plt::grid(true);
        // plt::xlabel("Number of (prediction + updates)");
        // plt::ylabel("Variance[m^2]");
        // // plt::figure();
        // plt::subplot(3,1,3);
        // plt::plot(updated_P2, "c-");
        // plt::title("Pzz , Variance of Z");
        // plt::grid(true);
        // plt::xlabel("Number of (prediction + updates)");
        // plt::ylabel("Variance[m^2]");
   
        // plt::pause(0.01); // 그래프 갱신
        plt::show();

        // // 창이 닫힐 때까지 대기
        // std::cout << "Press Enter to continue..." << std::endl;
        // std::cin.get();

    }

    void spin() {
        ros::Rate rate(100); 
        // double dt = 0.0025; // 0.05 second

        while (ros::ok()) {
            // addGaussianNoise(current_x,current_y,noise_x,noise_y);
            if (received_imu == true || received_velocity == true){
                
                // current_time= ros::Time::now().toSec();
                dt = current_time - last_predict_time;
                if(dt > 1){
                    dt = 0.0;
                }
                kalman_filter.predict(dt, velocity, yaw_rate);
                
                kalman_filter.getP();
                received_imu = false;
                received_velocity = false;
                last_predict_time = current_time;
                ROS_INFO("******Predict******");
                // -------------------------* 나중에 주석 처리 *---------------------------------------------//
                // predicted_covariance = kalman_filter.getP();
                // updated_P0.push_back(predicted_covariance(0,0));
                // updated_P1.push_back(predicted_covariance(1,1));
                // updated_P2.push_back(predicted_covariance(2,2));
                // -------------------------* 나중에 주석 처리 *---------------------------------------------//
            }
            if (received_gps == true) {
                // addGaussianNoise(current_x,current_y,noise_x,noise_y);
                // current_time =ros::Time::now().toSec();
                kalman_filter.update(Eigen::Vector3d(current_x, current_y, ego.heading));
                // kalman_filter.update(Eigen::Vector3d(noise_x, noise_y, ego.heading));
                // received_gps = false;
                // last_predict_time = current_time;
                // ROS_INFO("(current_x, current_y) : %.4f , %.4f", current_x, current_y);

                filtered_state = kalman_filter.getState();

                // -------------------------* 나중에 주석 처리 *---------------------------------------------//
                // updated_covariance = kalman_filter.getP();
                // updated_P0.push_back(updated_covariance(0,0));
                // updated_P1.push_back(updated_covariance(1,1));
                // updated_P2.push_back(updated_covariance(2,2));
                // // 시각화 데이터 업데이트
                // original_xs.push_back(current_x);
                // original_ys.push_back(current_y);
                // noisy_xs.push_back(noise_x);
                // noisy_ys.push_back(noise_y);

                // filtered_xs.push_back(filtered_state(0));
                // filtered_ys.push_back(filtered_state(1));
                // // RMSE 계산
                // rmse_original.push_back(calculateRMSE(original_xs, original_ys, noisy_xs, noisy_ys));
                // rmse_filtered.push_back(calculateRMSE(original_xs, original_ys, filtered_xs, filtered_ys));
                // -------------------------* 나중에 주석 처리 *---------------------------------------------//

                ego.x = filtered_state(0);
                ego.y = filtered_state(1);
                // ego.heading = filtered_state(2);

                publishVehicleState();
                // plotResults();
                ROS_INFO("----Update----");
                received_gps = false;
                last_predict_time = current_time;
            }
            ROS_INFO("dt : %.7f", dt);
            ROS_INFO("YAW : %.5f , Yaw_Rate : %.8f, Velocity :%.3f", ego.heading, yaw_rate, velocity);

            ros::spinOnce(); // 콜백 함수 호출
            rate.sleep();    // 루프 주기 유지
        }
    }

private:
    ros::Subscriber gps_sub;
    ros::Subscriber vehicle_status_sub;
    ros::Subscriber velocity_sub;
    ros::Publisher state_pub;
    //tfpub
    tf2_ros::TransformBroadcaster tf_broadcaster;

    EgoState ego;
    KalmanFilter kalman_filter;
    Eigen::Vector2d last_acceleration = Eigen::Vector2d::Zero(); // 마지막 가속도
    Eigen::Vector3d filtered_state = Eigen::Vector3d::Zero();
    std::default_random_engine generator; // 난수 생성기
    std::normal_distribution<double> distribution; // 노이즈 분포

    bool received_gps, received_imu, received_velocity;
    double current_x, current_y; // Carla Map 좌표
    double noise_x, noise_y;

    double yaw_rate;

    double current_time = 0.0;
    double last_predict_time = 0.0;
    double dt = 0.0;
    double velocity ;

    std::vector<double> rmse_original;
    std::vector<double> rmse_filtered;

    std::vector<double> yaw_rate_s;
    std::vector<double> velocity_s;
    std::vector<double> dt_s;

    Eigen::Matrix3d predicted_covariance;
    Eigen::Matrix3d updated_covariance;



    std::vector<double> updated_P0;
    std::vector<double> updated_P1;
    std::vector<double> updated_P2;

 

    // 시각화 데이터를 저장할 벡터
    std::vector<double> original_xs, original_ys;
    std::vector<double> filtered_xs, filtered_ys;
    std::vector<double> noisy_xs, noisy_ys;
    std::vector<double> errors_filtered; // 오차 저장 벡터
    std::vector<double> reference_xs, reference_ys; // 참조 경로 저장 벡터

    void cb_gps(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->latitude != 0 && msg->longitude != 0) {
            double latitude = msg->latitude;
            double longitude = msg->longitude;
            double altitude = msg->altitude;

            // Carla Map Coordinates로 변환
            Vector3d carla_coords = gnss_to_carla(latitude, longitude, altitude);
            current_x = carla_coords(0);
            current_y = carla_coords(1);

            received_gps = true;
            current_time = msg -> header.stamp.toSec();
            ROS_INFO("현재 시간 : %.7f", current_time);
        }
    }

    void cb_vehicle_status(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg) {
        // Orientation에서 Yaw 값을 추출
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        ego.heading = -yaw;
        while(ego.heading> M_PI){
            ego.heading -= 2 * M_PI;
        }
        while(ego.heading < - M_PI){
            ego.heading += 2 * M_PI;
        }
        yaw_rate = -(msg -> acceleration.angular.z);    // 왼손좌표계이니
        // // vehicle_status 토픽에서 velocity 값 사용
        velocity = msg->velocity;
        // ROS_INFO("YAW : %.2f, velocity : %.2f", ego.heading, velocity);
        received_imu = true;
        received_velocity = true;
        current_time = msg -> header.stamp.toSec();
        
    }

    void cb_imu(const sensor_msgs::Imu::ConstPtr& msg){
        // Orientation에서 Yaw 값을 추출
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        ego.heading = -yaw;
        while(ego.heading> M_PI){
            ego.heading -= 2 * M_PI;
        }
        while(ego.heading < - M_PI){
            ego.heading += 2 * M_PI;
        }
       
        yaw_rate = -(msg -> angular_velocity.z); 
        
        yaw_rate_s.push_back(static_cast<double>(yaw_rate));
        received_imu = true;
        current_time = msg -> header.stamp.toSec();

    }

    
    void cb_velocity(const carla_msgs::CarlaSpeedometer::ConstPtr& msg){
        velocity = (msg->speed);   
        received_velocity = true;
        velocity_s.push_back(static_cast<double>(velocity));

    }

    void addGaussianNoise(double original_x, double original_y, double &noisy_x, double &noisy_y) {
        noisy_x = original_x + distribution(generator);
        noisy_y = original_y + distribution(generator);
    }

    void initializeReferencePath() {
        reference_xs = {
            6770.899414, 6770.933594, 6770.975586, 6771.009766, 6771.043457, 6771.085938,
            6771.120117, 6771.153809, 6771.187988, 6771.221680, 6771.255859, 6771.289551,
            6771.332031, 6771.366211, 6771.399902, 6771.442383, 6771.476562, 6771.510254,
            6771.552734, 6771.586914, 6771.620605, 6771.663086, 6771.697266, 6771.730957,
            6771.773438, 6771.807617, 6771.841309, 6771.875488, 6771.909180, 6771.943359,
            6771.977051, 6772.019531, 6772.053711, 6772.087402, 6772.129883, 6772.163574,
            6772.197754, 6772.240234, 6772.273926, 6772.308105, 6772.341797, 6772.375977,
            6772.409668, 6772.443848, 6772.486328, 6772.520020
        };
        reference_ys = {
            5426.435547, 5422.435547, 5417.436035, 5413.436035, 5409.436035, 5404.436523,
            5400.436523, 5396.436523, 5392.437012, 5388.437012, 5384.437012, 5380.437012,
            5375.437500, 5371.437500, 5367.437500, 5362.437988, 5358.437988, 5354.437988,
            5349.438477, 5345.438477, 5341.438477, 5336.438965, 5332.438965, 5328.438965,
            5323.439453, 5319.439453, 5315.439453, 5311.439941, 5307.439941, 5303.439941,
            5299.439941, 5294.440430, 5290.440430, 5286.440430, 5281.440918, 5277.440918,
            5273.440918, 5268.441406, 5264.441406, 5260.441406, 5256.441895, 5252.441895,
            5248.441895, 5244.441895, 5239.442383, 5235.442383
        };
    }

    double calculateRMSE(const std::vector<double>& true_values_xs, const std::vector<double>& true_values_ys,  const std::vector<double>& predicted_values_xs, const std::vector<double>& predicted_values_ys) {
        if (true_values_xs.size() != predicted_values_xs.size() || true_values_xs.empty()) {
            ROS_ERROR("Vectors must be of the same size and non-empty");
            return 0.0;
        }

        double sum_squared_errors = 0.0;
        for (size_t i = 0; i < true_values_xs.size(); ++i) {
            double error_x = true_values_xs[i] - predicted_values_xs[i];
            double error_y = true_values_ys[i] - predicted_values_ys[i];
            double error = std::sqrt(error_x * error_x + error_y * error_y); // Euclidean distance
            sum_squared_errors += error;
        }
        double mean_squared_error = sum_squared_errors / true_values_xs.size();
        return mean_squared_error;
    }


    // //tfpub
    // void publishTf(const double& x,const double& y, const double& yaw ){
    //     geometry_msgs::TransformStamped transformStamped;
        
    //     transformStamped.header.stamp = ros::Time::now();
    //     transformStamped.header.frame_id = "map";  // 부모 프레임
    //     transformStamped.child_frame_id = "hero"; // 자식 프레임

    //     // 변환 설정 (위치 및 회전)
    //     transformStamped.transform.translation.x = x;
    //     transformStamped.transform.translation.y = y;
    //     transformStamped.transform.translation.z = 0.0;

    //     tf2::Quaternion quat;
    //     quat.setRPY(0, 0, -yaw);  // 회전을 쿼터니언으로 설정 (롤, 피치, 요)
    //     transformStamped.transform.rotation.x = quat.x();
    //     transformStamped.transform.rotation.y = quat.y();
    //     transformStamped.transform.rotation.z = quat.z();
    //     transformStamped.transform.rotation.w = quat.w();

    //     // TF 발행
    //     tf_broadcaster.sendTransform(transformStamped);
    // }
    void publishVehicleState() {
        custom_msgs::vehicle_state state_msg;
        state_msg.x = ego.x;
        state_msg.y = ego.y;
        state_msg.heading = ego.heading;
        state_pub.publish(state_msg);
        //tfpub
        // publishTf(ego.x,ego.y,ego.heading);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "localization_node");
    Localization localization;
    localization.spin(); // 주기적으로 데이터를 처리하는 메서드
    // localization.plotResults();
    std::cout << "Press Enter to stop visualization..." << std::endl;
    std::cin.get(); // Wait for Enter key press
    return 0;
}
