
#ifndef SENSOR_ODOM_2D_H_
#define SENSOR_ODOM_2D_H_

//wolf includes
#include "sensor_base.h"
#include "sensor_factory.h"

namespace wolf {

class SensorOdom2D : public SensorBase
{

    protected:
        Scalar k_disp_to_disp_; ///< ratio of displacement variance to displacement, for odometry noise calculation
        Scalar k_rot_to_rot_; ///< ratio of rotation variance to rotation, for odometry noise calculation

	public:
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * \param _p_ptr StateBlock pointer to the sensor position wrt vehicle base
         * \param _o_ptr StateBlock pointer to the sensor orientation wrt vehicle base
         * \param _disp_noise_factor displacement noise factor
         * \param _rot_noise_factor rotation noise factor
         * 
         **/
		SensorOdom2D(StateBlock* _p_ptr, StateBlock* _o_ptr, const Scalar& _disp_noise_factor, const Scalar&  _rot_noise_factor);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/
        virtual ~SensorOdom2D();
        
        /** \brief Returns displacement noise factor
         * 
         * Returns displacement noise factor
         * 
         **/        
        double getDispVarToDispNoiseFactor() const;

        /** \brief Returns rotation noise factor
         * 
         * Returns rotation noise factor
         * 
         **/        
        double getRotVarToRotNoiseFactor() const;
        
};

// Define the factory method and register it in the SensorFactory
namespace
{
SensorBase* createOdom2D(std::string& _name)
{
    SensorBase* odo = new SensorOdom2D(nullptr, nullptr,0,0);
    odo->setName(_name);
    return odo;
}
const bool registered_odom_2d = SensorFactory::get()->registerSensor("ODOM_2D", createOdom2D);
}


} // namespace wolf

#endif
