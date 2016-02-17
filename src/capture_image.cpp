#include "capture_image.h"

CaptureImage::CaptureImage(const TimeStamp& _ts, SensorCamera* _camera_ptr, const Eigen::MatrixXs& _data, int _img_width, int _img_height) :
    CaptureBase(_ts, _camera_ptr),
    data_(_data)
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

//void CaptureGPSFix::printSelf(unsigned int _ntabs, std::ostream & _ost) const
//{
//    NodeLinked::printSelf(_ntabs, _ost);
//    //printTabs(_ntabs);
//    //_ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
//    //printNTabs(_ntabs);
//    //_ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
//}


