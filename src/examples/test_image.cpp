// Testing things for the 3D image odometry

//std includes
#include <iostream>
#include <memory>
#include <random>
#include <cmath>
#include <queue>

//Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"
#include "feature_point.h"

int main(int argc, char** argv)
{
    //Welcome message
    std::cout << std::endl << " ========= WOLF IMAGE test ===========" << std::endl << std::endl;

    int img_width = 640;
    int img_height = 480;

    //SensorCamera test
    std::cout << std::endl << " ========= SensorCamera test ===========" << std::endl << std::endl;

    Eigen::Vector4s k = {320,240,320,320};

    StateBlock* intr = new StateBlock(k,false);

    SensorCamera* sen_cam_ = new SensorCamera(new StateBlock(Eigen::Vector3s::Zero()), new StateBlock(Eigen::Vector3s::Zero()),
                                              intr,img_width,img_height);
    std::cout << " image sensor created!" << std::endl;
    std::cout << " intr_test" << std::endl << intr->getVector() << std::endl;
    std::cout << " sen_cam intr_ptr" << std::endl << sen_cam_->getIntrinsicPtr()->getVector() << std::endl;



    //CaptureImage test
    std::cout << std::endl << " ========= CaptureImage test ===========" << std::endl << std::endl;

    Eigen::MatrixXs data(img_height,img_width); //first height, last lenght
    data(0,0)=12;
    TimeStamp t = 1;

    CaptureImage* cap_image = new CaptureImage(t,sen_cam_,data,img_width,img_height);

    std::cout << " timestamp: " << t.get() << std::endl;
    std::cout << " data: " << data(0,0) << " " << data(1,0) << " " << data(2,0) << std::endl;
    std::cout << " captureimage test: " << cap_image->getSensorPtr()->getIntrinsicPtr()->getVector() << std::endl;
    //std::cout << " data: " << cap_image->data_.cols() << " " << cap_image->data_.rows()<< std::endl;




    //FeaturePoint test
    std::cout << std::endl << " ========= FeaturePoint test ===========" << std::endl << std::endl;

    Eigen::Vector2s m_pos = {5,6};
    Eigen::Matrix2s m_cov(2,2);
    m_cov(0,0)=1;
    m_cov(0,1)=2;
    m_cov(1,0)=3;
    m_cov(1,1)=4;

    FeaturePoint* feat_point = new FeaturePoint(m_pos,m_cov);
    std::cout << " measurement position: " << m_pos(0) << " " << m_pos(1) << std::endl;
    std::cout << " measurement covariance:\n " << m_cov(0,0) << " " << m_cov(0,1) << "\n " << m_cov(1,0) << " " << m_cov(1,1) << std::endl;
    std::cout << feat_point->getMeasurement() << "\n" << feat_point->getMeasurementCovariance()<< std::endl;
}
