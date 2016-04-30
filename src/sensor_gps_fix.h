
#ifndef SENSOR_GPS_FIX_H_
#define SENSOR_GPS_FIX_H_

//wolf includes
#include "sensor_base.h"
#include "sensor_factory.h"

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

// Define the factory method and register it in the SensorFactory
namespace
{
SensorBase* createGPSFix(std::string& _name, std::string _params_filename = "")
{
    SensorBase* sen = new SensorGPSFix(nullptr, nullptr, 0);
    sen->setName(_name);
    return sen;
}
const bool registered_gps_fix = SensorFactory::get()->registerSensor("GPS_FIX", createGPSFix);
}
} // namespace wolf

#endif
