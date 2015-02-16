
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
         * \param _sp sensor 3D pose with respect to vehicle base frame
         * \param _noise noise standard deviation
         * 
         **/
		SensorGPSFix(const Eigen::VectorXs & _sp, const double& _noise);
        
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
