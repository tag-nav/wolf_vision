
#ifndef SENSOR_TWIST_2D_H_
#define SENSOR_TWIST_2D_H_

//wolf includes
#include "sensor_base.h"

class SensorTwist2D : public SensorBase
{
    public:
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * \param _p_ptr StateBase pointer to the sensor position wrt vehicle base
         * \param _o_ptr StateOrientation pointer to the sensor orientation wrt vehicle base
         * \param _lineal_noise lineal velocity noise
         * \param _angular_noise angular velocity noise
         * 
         **/
        SensorTwist2D(StatePoint3D* _p_ptr, StateOrientation* _o_ptr, const WolfScalar& _lineal_noise, const WolfScalar&  _angular_noise);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~SensorTwist2D();
        
        /** \brief Returns displacement noise factor
         * 
         * Returns displacement noise factor
         * 
         **/        
        double getLinealNoise() const;

        /** \brief Returns rotation noise factor
         * 
         * Returns rotation noise factor
         * 
         **/        
        double getAngularNoise() const;
        
};
#endif
