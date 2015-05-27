
#ifndef SENSOR_ODOM_2D_H_
#define SENSOR_ODOM_2D_H_

//wolf includes
#include "sensor_base.h"

class SensorOdom2D : public SensorBase
{
    public:
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * \param _p_ptr StateBase pointer to the sensor position wrt vehicle base
         * \param _o_ptr StateOrientation pointer to the sensor orientation wrt vehicle base
         * \param _disp_noise_factor displacement noise factor
         * \param _rot_noise_factor rotation noise factor
         * 
         **/
		SensorOdom2D(StatePoint3D* _p_ptr, StateOrientation* _o_ptr, const WolfScalar& _disp_noise_factor, const WolfScalar&  _rot_noise_factor);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~SensorOdom2D();
        
        /** \brief Returns displacement noise factor
         * 
         * Returns displacement noise factor
         * 
         **/        
        double getDisplacementNoiseFactor() const;

        /** \brief Returns rotation noise factor
         * 
         * Returns rotation noise factor
         * 
         **/        
        double getRotationNoiseFactor() const;
        
};
#endif
