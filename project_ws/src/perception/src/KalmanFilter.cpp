#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
    // F = Eigen::Matrix3d::Identity();
    F = Eigen::Matrix3d::Identity();
    H = Eigen::Matrix3d::Identity(); // 관측 행렬을 단위행렬로 설정
    Q = Eigen::Matrix3d::Identity()*0.0001;
    // Q(2,2) = 0.0001;
    G = Eigen::Matrix<double, 3, 2>::Zero(); // G를 초기화    
    u = Eigen::Vector2d::Zero();
    R = Eigen::Matrix3d::Identity() * 1;    // R은 고정
    R(2,2) = 1 * M_PI / 180;
    P = Eigen::Matrix3d::Identity()*1000;
    x = Eigen::Vector3d::Zero();
}

void KalmanFilter::init(const Eigen::Vector3d& initial_state) {
    x = initial_state;
}

void KalmanFilter::predict(double dt, double v, double yaw_rate) {
    Q = Eigen::Matrix3d::Identity()*0.0001;
    // G(0,0) = v * dt * cos(x(2));     
    // G(1,0) = v * dt * sin(x(2));
    // G(2,1) = dt;
    
    // u(0) = v;
    // u(1) = yaw_rate;
    
    F(0,2) = -v*dt*sin(x(2));
    F(1,2) = v*dt*cos(x(2));

    // 상태 예측
    x(0) += v *dt * cos(x(2));
    x(1) += v *dt * sin(x(2));
    x(2) += yaw_rate *dt;
    if (abs(yaw_rate) > 0.08){
        Q(0,0) += abs(yaw_rate * dt);
        Q(1,1) += abs(yaw_rate * dt);
        Q(2,2) += abs(yaw_rate * dt);
    }

    // 오차 공분산 행렬 업데이트
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::Vector3d& measurement) {
    // 예측 오차 계산
    
    Eigen::Vector3d y = measurement - H * x;
    while(y(2)> M_PI){
        y(2) -= 2 * M_PI;
    }
    while(y(2) < - M_PI){
        y(2) += 2 * M_PI;
    }
    

    Eigen::Matrix3d S = H * P * H.transpose() + R;
    Eigen::Matrix3d K = P * H.transpose() * S.inverse();

    // 상태 벡터 업데이트
    x = x + K * y;

    // 오차 공분산 행렬 업데이트
    P = (Eigen::Matrix3d::Identity() - K * H) * P;
}

Eigen::Vector3d KalmanFilter::getState() const {
    return x;
}

Eigen::Matrix3d KalmanFilter::getP() const{
    return P;
}

KalmanFilter2::KalmanFilter2() {
    // F = Eigen::Matrix3d::Identity();
    F = Eigen::Matrix4d::Identity();
    H = Eigen::Matrix<double, 2, 4>::Zero(); // 관측 행렬을 단위행렬로 설정
    H(0,0) = 1;
    H(1,1) = 1;  
    Q = Eigen::Matrix4d::Identity()* 0.0001;
    R = Eigen::Matrix2d::Identity() * 0.01;    // R은 고정
    P = Eigen::Matrix4d::Identity()*100;
    x = Eigen::Vector4d::Zero();
}

void KalmanFilter2::init(const Eigen::Vector4d& initial_state) {
    x = initial_state;
}

void KalmanFilter2::predict(double dt) {

    
    F(0,2) = dt;
    F(1,3) = dt;

    // 상태 예측
    x = F * x;
   
    
    // 오차 공분산 행렬 업데이트
    P = F * P * F.transpose() + Q;
}

void KalmanFilter2::update(const Eigen::Vector2d& measurement) {
    // 예측 오차 계산
    Eigen::Vector2d y = measurement - H * x;

    Eigen::Matrix2d S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();

    // 상태 벡터 업데이트
    x = x + K * y;

    // 오차 공분산 행렬 업데이트
    P = (Eigen::Matrix4d::Identity() - K * H) * P;
}

Eigen::Vector4d KalmanFilter2::getState() const {
    return x;
}

Eigen::Matrix4d KalmanFilter2::getP() const{
    return P;
}
