
#ifndef SENSOR_GPS_FIX_H_
#define SENSOR_GPS_FIX_H_

//wolf includes
#include "sensor_base.h"

class SensorGPSFix : public SensorBase
{
    public:
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * \param _p_ptr StateBase pointer to the sensor position
         * \param _o_ptr StateBase pointer to the sensor orientation
         * \param _noise noise standard deviation
         * 
         **/
		SensorGPSFix(StateBlock* _p_ptr, StateBlock* _o_ptr, const double& _noise);
        
        /** \brief Destructor
         * 
         * Destructor
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
#endif
