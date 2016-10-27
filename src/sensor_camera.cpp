#include "sensor_camera.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "pinholeTools.h"

namespace wolf
{

SensorCamera::SensorCamera(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _intr_ptr, //
                           int _img_width, int _img_height) :
        SensorBase(SEN_CAMERA, "CAMERA", _p_ptr, _o_ptr, _intr_ptr, 2), //
        img_width_(_img_width), img_height_(_img_height)
{
    //
}

SensorCamera::SensorCamera(const Eigen::VectorXs& _extrinsics, const std::shared_ptr<IntrinsicsCamera> _intrinsics_ptr) :
                SensorBase(SEN_CAMERA, "CAMERA", std::make_shared<StateBlock>(_extrinsics.head(3)), std::make_shared<StateQuaternion>(_extrinsics.tail(4)), std::make_shared<StateBlock>(_intrinsics_ptr->pinhole_model), 1),
                img_width_(_intrinsics_ptr->width), //
                img_height_(_intrinsics_ptr->height), //
                distortion_(_intrinsics_ptr->distortion), //
                correction_(distortion_.size()+2) // make correction vector of the same size as distortion vector
{
    assert(_extrinsics.size() == 7 && "Wrong intrinsics vector size. Should be 7 for 3D");
    K_ = setIntrinsicMatrix(_intrinsics_ptr->pinhole_model);
    pinhole::computeCorrectionModel(getIntrinsicPtr()->getVector(), distortion_, correction_);
    std::cout << "\tintrinsics  : " << getIntrinsicPtr()->getVector().transpose() << std::endl;
    std::cout << "\tintrinsic matrix  : " << K_ << std::endl;
//    std::cout << "\tdistortion  : " << distortion_.transpose() << std::endl;
//    std::cout << "\tcorrection  : " << correction_.transpose() << std::endl;
    std::cout << "\tp_ptr  : " << getPPtr()->getVector().transpose() << std::endl;
    std::cout << "\to_ptr  : " << getOPtr()->getVector().transpose() << std::endl;
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
    std::shared_ptr<SensorCamera> sen_ptr = std::make_shared<SensorCamera>(_extrinsics_pq, intrinsics_ptr);
    sen_ptr->setName(_unique_name);

    return sen_ptr;
}


} // namespace wolf

// Register in the SensorFactory
#include "sensor_factory.h"
//#include "factory.h"
namespace wolf
{
WOLF_REGISTER_SENSOR("CAMERA", SensorCamera)
} // namespace wolf

