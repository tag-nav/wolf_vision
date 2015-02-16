
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
         * \param _sp sensor 3D pose with respect to vehicle base frame
         * \param _disp_noise_factor displacement noise factor
         * \param _rot_noise_factor rotation noise factor
         * 
         **/
		SensorOdom2D(const Eigen::VectorXs & _sp, const WolfScalar& _disp_noise_factor, const WolfScalar&  _rot_noise_factor);
        
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
