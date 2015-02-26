
#ifndef CAPTURE_LASER_2D_H_
#define CAPTURE_LASER_2D_H_

//std includes
#include <queue>
#include <random>

// Eigen ingludes
#include <eigen3/Eigen/Geometry>

//wolf includes
#include "capture_base.h"
#include "sensor_laser_2D.h"
#include "feature_corner_2D.h"
#include "landmark_corner_2D.h"
#include "state_point.h"

//wolf forward declarations
//#include "feature_corner_2D.h"

struct Line{
	Eigen::Vector3s vector;
	WolfScalar error;
	unsigned int last, first;
};

class CaptureLaser2D : public CaptureBase
{
    protected:
        static unsigned int segment_window_size;//window size to extract segments
        static double theta_min; //minimum theta between consecutive segments to detect corner. PI/6=0.52
        static double theta_max_parallel; //maximum theta between consecutive segments to fuse them in a single line.
        static double k_sigmas;//How many std_dev are tolerated to count that a point is supporting a line
        static unsigned int max_beam_distance;//max number of beams of distance between lines to consider corner or concatenation
        static double max_distance;//max distance between line ends to consider corner or concatenation
        Eigen::Map<Eigen::VectorXs> ranges_; // a map to the ranges inside de data vector
        Eigen::Map<Eigen::VectorXs> intensities_; // a map to the intensities inside the data vector
        
    public:
        /** \brief Constructor with ranges
         * 
         * Constructor with ranges
         * 
         **/
        CaptureLaser2D(const TimeStamp & _ts, const SensorLaser2DPtr & _sensor_ptr, const Eigen::VectorXs& _ranges);

        /** \brief Constructor with ranges and intensities
         *
         * Constructor with ranges and intensities
         *
         **/
        CaptureLaser2D(const TimeStamp & _ts, const SensorLaser2DPtr & _sensor_ptr, const Eigen::VectorXs& _ranges, const Eigen::VectorXs& _intensities);

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
        virtual unsigned int extractCorners(std::list<Eigen::Vector4s> & _corner_list) const;

        /** \brief get corners
         *
         * Get corners
         *
         **/
        //std::list<Eigen::Vector4s> getCorners() const;
        
        void fitLine(unsigned int _idx_from, unsigned int _idx_to, const Eigen::MatrixXs& _points, Line& line_) const;

        virtual void createFeatures(std::list<Eigen::Vector4s> & _corner_list); //TODO: should be const .... JVN: No, because new feature is added to the list
        
        void establishConstraints();

        virtual Eigen::VectorXs computePrior() const;
};
#endif /* CAPTURE_LASER_2D_H_ */
