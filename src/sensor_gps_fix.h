
#ifndef SENSOR_GPS_FIX_H_
#define SENSOR_GPS_FIX_H_

//wolf includes
#include "sensor_base.h"

// std includes


namespace wolf {

class SensorGPSFix : public SensorBase
{
    public:
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _noise noise standard deviation
         * 
         **/
		SensorGPSFix(StateBlock* _p_ptr, StateBlock* _o_ptr, const double& _noise);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/
        virtual ~SensorGPSFix();
        
        /** \brief Returns noise standard deviation
         * 
         * Returns noise standard deviation
         * 
         **/        
        double getNoise() const;
        
};

} // namespace wolf

#include "sensor_factory.h"

namespace wolf {

// Define the factory method and register it in the SensorFactory
namespace
{
SensorBase* createGPSFix(std::string& _name, Eigen::VectorXs& _extrinsics, IntrinsicsBase* _intrinsics)
{
    SensorBase* sen = new SensorGPSFix(nullptr, nullptr, 0);
    sen->setName(_name);
    return sen;
}
const bool registered_gps_fix = SensorFactory::get()->registerCreator("GPS FIX", createGPSFix);
}
} // namespace wolf

#endif
