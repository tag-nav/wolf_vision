
#include "sensor_gps.h"
#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

SensorGPS::SensorGPS(StateBlock* _p_ptr, //GPS sensor position with respect to the car frame (base frame)
                     StateBlock* _o_ptr, //GPS sensor orientation with respect to the car frame (base frame)
                     StateBlock* _bias_ptr,  //GPS sensor time bias
                     StateBlock* _map_p_ptr, //initial position of vehicle's frame with respect to starting point frame
                     StateBlock* _map_o_ptr) //initial orientation of vehicle's frame with respect to the starting point frame
        :
        SensorBase(SEN_GPS_RAW, _p_ptr, _o_ptr, _bias_ptr, 0),
        map_p_ptr_(_map_p_ptr),
        map_o_ptr_(_map_o_ptr)
{
    setType("GPS");
}

// TODO Check carefully this destructor!
SensorGPS::~SensorGPS()
{

}

StateBlock *SensorGPS::getMapPPtr() const
{
    return map_p_ptr_;
}

StateBlock *SensorGPS::getMapOPtr() const
{
    return map_o_ptr_;
}

void SensorGPS::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        if (p_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(p_ptr_);

        if (o_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(o_ptr_);

        if (intrinsic_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(intrinsic_ptr_);

        if (map_p_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(map_p_ptr_);

        if (map_o_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(map_o_ptr_);
    }
}

// Define the factory method
SensorBase* SensorGPS::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_p,
                              const IntrinsicsBase* _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_p.size() == 3 && "Bad extrinsics vector length. Should be 3 for 3D.");
    StateBlock* pos_ptr = new StateBlock(_extrinsics_p, true);
    StateBlock* ori_ptr = nullptr;
    SensorBase* sen = new SensorGPS(pos_ptr, ori_ptr, nullptr, nullptr, nullptr); // TODO: how to init these last three pointers?
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf
