#include "capture_image.h"
#include <opencv2/core/core.hpp>

CaptureImage::CaptureImage(const TimeStamp& _ts, SensorCamera* _camera_ptr, cv::Mat _data_cv, int _img_width, int _img_height) :
    CaptureBase(_ts, _camera_ptr), image_(_data_cv)
{
    assert((_img_width == _camera_ptr->img_width_ && _img_height == _camera_ptr->img_height_) && "Image and camera sizes don't match");
}

CaptureImage::~CaptureImage()
{
    //
}

Eigen::VectorXs CaptureImage::computeFramePose(const TimeStamp& _now) const
{

    return Eigen::VectorXs::Zero(7);
}

cv::Mat CaptureImage::getImage()
{
    return image_;
}

//void CaptureGPSFix::printSelf(unsigned int _ntabs, std::ostream & _ost) const
//{
//    NodeLinked::printSelf(_ntabs, _ost);
//    //printTabs(_ntabs);
//    //_ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
//    //printNTabs(_ntabs);
//    //_ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
//}


