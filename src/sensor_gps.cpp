
#include "sensor_gps.h"

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
    setType("GPS RAW");
    //std::cout << "SensorGPS constructor... id: " << nodeId() << std::endl;
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

} // namespace wolf
