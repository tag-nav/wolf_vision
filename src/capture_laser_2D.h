
#ifndef CAPTURE_LASER_2D_H_
#define CAPTURE_LASER_2D_H_

//wolf includes
#include "capture_base.h"
#include "sensor_laser_2D.h"

//wolf forward declarations
//#include "feature_corner_2D.h"

class CaptureLaser2D : public CaptureBase
{
    protected:
        static unsigned int segment_window_size;//window size to extract segments
        static double theta_min; //minimum theta between consecutive segments to detect corner. PI/6=0.52
        static double k_sigmas;//How many std_dev are tolerated to count that a point is supporting a line
        
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
        virtual unsigned int extractCorners(std::list<Eigen::Vector2s> & _corner_list);
        
        virtual Eigen::VectorXs computePrior() const;
        virtual void findCorrespondences();
};
#endif /* CAPTURE_LASER_2D_H_ */
