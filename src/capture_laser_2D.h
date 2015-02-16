
#ifndef CAPTURE_LASER_2D_H_
#define CAPTURE_LASER_2D_H_

//wolf includes
#include "capture_base.h"
#include "sensor_laser_2D.h"

//wolf forward declarations
//#include "feature_corner_2D.h"

class CaptureLaser2D : public CaptureBase
{
    public:
        /** \brief Constructor
         * 
         * Constructor
         * 
         **/
        CaptureLaser2D(const TimeStamp & _ts, const SensorLaser2DPtr & _sensor_ptr, const Eigen::VectorXs& _ranges);

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/        
        virtual ~CaptureLaser2D();
        
        /** \brief Calls the necessary pipeline from raw scan to features.
         * 
         * Calls the necessary pipeline from raw scan to features.
         * 
         **/
        virtual void processCapture();

        /** \brief Extract corners and push-back to Feature down list 
         * 
         * Extract corners and push-back to Feature down list . 
         * 
         **/
        virtual void extractCorners();
};
#endif /* CAPTURE_LASER_2D_H_ */
