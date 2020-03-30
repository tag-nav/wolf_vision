
#include "vision/sensor/sensor_camera.h"
#include "vision/math/pinhole_tools.h"

#include "core/state_block/state_block.h"
#include "core/state_block/state_quaternion.h"

namespace wolf
{

SensorCamera::SensorCamera(const Eigen::VectorXd& _extrinsics, const ParamsSensorCamera& _intrinsics) :
                SensorBase("CAMERA", std::make_shared<StateBlock>(_extrinsics.head(3), true), std::make_shared<StateQuaternion>(_extrinsics.tail(4), true), std::make_shared<StateBlock>(_intrinsics.pinhole_model_raw, true), 1),
                img_width_(_intrinsics.width), //
                img_height_(_intrinsics.height), //
                distortion_(_intrinsics.distortion), //
                correction_(distortion_.size()==0 ? 0 : distortion_.size() + 1), // make correction vector slightly larger in size than the distortion vector
                pinhole_model_raw_(_intrinsics.pinhole_model_raw), //
                pinhole_model_rectified_(_intrinsics.pinhole_model_rectified), //
                using_raw_(true)
{
    assert(_extrinsics.size() == 7 && "Wrong intrinsics vector size. Should be 7 for 3d");
    useRawImages();
    pinhole::computeCorrectionModel(getIntrinsic()->getState(), distortion_, correction_);
}

SensorCamera::SensorCamera(const Eigen::VectorXd& _extrinsics, ParamsSensorCameraPtr _intrinsics_ptr) :
        SensorCamera(_extrinsics, *_intrinsics_ptr)
{
    //
}

SensorCamera::~SensorCamera()
{
    //
}

Eigen::Matrix3d SensorCamera::setIntrinsicMatrix(Eigen::Vector4d _pinhole_model)
{
    Eigen::Matrix3d K;
    K(0, 0) = _pinhole_model(2);
    K(0, 1) = 0;
    K(0, 2) = _pinhole_model(0);
    K(1, 0) = 0;
    K(1, 1) = _pinhole_model(3);
    K(1, 2) = _pinhole_model(1);
    K.row(2) << 0, 0, 1;
    return K;
}

// Define the factory method
SensorBasePtr SensorCamera::create(const std::string& _unique_name, //
                                 const Eigen::VectorXd& _extrinsics_pq, //
                                 const ParamsSensorBasePtr _intrinsics)
{
    assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3d.");

    std::shared_ptr<ParamsSensorCamera> intrinsics_ptr = std::static_pointer_cast<ParamsSensorCamera>(_intrinsics);
    SensorCameraPtr sen_ptr = std::make_shared<SensorCamera>(_extrinsics_pq, intrinsics_ptr);
    sen_ptr->setName(_unique_name);

    return sen_ptr;
}

} // namespace wolf

// Register in the SensorFactory
#include "core/sensor/sensor_factory.h"
namespace wolf
{
WOLF_REGISTER_SENSOR("CAMERA", SensorCamera)
} // namespace wolf

