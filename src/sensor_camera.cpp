#include "sensor_camera.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "pinholeTools.h"

namespace wolf
{

SensorCamera::SensorCamera(StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr, //
                           int _img_width, int _img_height) :
        SensorBase(SEN_CAMERA, _p_ptr, _o_ptr, _intr_ptr, 2), //
        img_width_(_img_width), img_height_(_img_height)
{
    setType("CAMERA");
}

SensorCamera::SensorCamera(const Eigen::VectorXs& _extrinsics, const IntrinsicsCamera* _intrinsics_ptr) :
                SensorBase(SEN_CAMERA, nullptr, nullptr, nullptr, 2), // will initialize state blocks later
                img_width_(_intrinsics_ptr->width), //
                img_height_(_intrinsics_ptr->height), //
                distortion_(_intrinsics_ptr->distortion), //
                correction_(distortion_.size()) // make correction vector of the same size as distortion vector
{
    assert(_extrinsics.size() == 7 && "Wrong intrinsics vector size. Should be 7 for 3D");
    setType("CAMERA");
    p_ptr_ = new StateBlock(_extrinsics.head(3));
    o_ptr_ = new StateQuaternion(_extrinsics.tail(4));
    intrinsic_ptr_ = new StateBlock(_intrinsics_ptr->pinhole_model);
    pinhole::computeCorrectionModel(intrinsic_ptr_->getVector(), distortion_, correction_);
    std::cout << "\tintrinsic_ptr  : " << intrinsic_ptr_->getVector().transpose() << std::endl;
    std::cout << "\tdistortion  : " << distortion_.transpose() << std::endl;
    std::cout << "\tcorrection  : " << correction_.transpose() << std::endl;
    std::cout << "\tp_ptr  : " << p_ptr_->getVector().transpose() << std::endl;
    std::cout << "\to_ptr  : " << o_ptr_->getVector().transpose() << std::endl;
}


SensorCamera::~SensorCamera()
{
    //
}

// Define the factory method
SensorBase* SensorCamera::create(const std::string& _unique_name, //
                                 const Eigen::VectorXs& _extrinsics_pq, //
                                 const IntrinsicsBase* _intrinsics)
{
    assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");

    IntrinsicsCamera* intrinsics_ptr = (IntrinsicsCamera*)_intrinsics;
    SensorCamera* sen_ptr = new SensorCamera(_extrinsics_pq, intrinsics_ptr);
    sen_ptr->setName(_unique_name);

    return sen_ptr;
}


} // namespace wolf

// Register in the SensorFactory
#include "sensor_factory.h"
//#include "factory.h"
namespace wolf
{
namespace
{
const bool registered_camera = SensorFactory::get().registerCreator("CAMERA", SensorCamera::create);
}
} // namespace wolf

