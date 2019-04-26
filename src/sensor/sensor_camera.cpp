#include "base/sensor/sensor_camera.h"

#include "base/math/pinhole_tools.h"
#include "base/state_block/state_block.h"
#include "base/state_block/state_quaternion.h"

namespace wolf
{

SensorCamera::SensorCamera(const Eigen::VectorXs& _extrinsics, const IntrinsicsCamera& _intrinsics) :
                SensorBase("CAMERA", std::make_shared<StateBlock>(_extrinsics.head(3), true), std::make_shared<StateQuaternion>(_extrinsics.tail(4), true), std::make_shared<StateBlock>(_intrinsics.pinhole_model_raw, true), 1),
                img_width_(_intrinsics.width), //
                img_height_(_intrinsics.height), //
                distortion_(_intrinsics.distortion), //
                correction_(distortion_.size() + 1), // make correction vector slightly larger in size than the distortion vector
                pinhole_model_raw_(_intrinsics.pinhole_model_raw), //
                pinhole_model_rectified_(_intrinsics.pinhole_model_rectified), //
                using_raw_(true)
{
    assert(_extrinsics.size() == 7 && "Wrong intrinsics vector size. Should be 7 for 3D");
    useRawImages();
    pinhole::computeCorrectionModel(getIntrinsic()->getState(), distortion_, correction_);
}

SensorCamera::SensorCamera(const Eigen::VectorXs& _extrinsics, IntrinsicsCameraPtr _intrinsics_ptr) :
        SensorCamera(_extrinsics, *_intrinsics_ptr)
{
    //
}

SensorCamera::~SensorCamera()
{
    //
}

Eigen::Matrix3s SensorCamera::setIntrinsicMatrix(Eigen::Vector4s _pinhole_model)
{
    Eigen::Matrix3s K;
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
                                 const Eigen::VectorXs& _extrinsics_pq, //
                                 const IntrinsicsBasePtr _intrinsics)
{
    assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");

    std::shared_ptr<IntrinsicsCamera> intrinsics_ptr = std::static_pointer_cast<IntrinsicsCamera>(_intrinsics);
    SensorCameraPtr sen_ptr = std::make_shared<SensorCamera>(_extrinsics_pq, intrinsics_ptr);
    sen_ptr->setName(_unique_name);

    return sen_ptr;
}

} // namespace wolf

// Register in the SensorFactory
#include "base/sensor/sensor_factory.h"
namespace wolf
{
WOLF_REGISTER_SENSOR("CAMERA", SensorCamera)
} // namespace wolf

