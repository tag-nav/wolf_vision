
#ifndef CAPTURE_LASER_2D_H_
#define CAPTURE_LASER_2D_H_

//std includes
#include <queue>
#include <random>
#include <math.h>
#include <algorithm> //find(), sort()

// Eigen ingludes
#include <eigen3/Eigen/Geometry>

//laser_scan_utils
#include "iri-algorithms/laser_scan_utils/scan_basics.h"
#include "iri-algorithms/laser_scan_utils/corner_detector.h"
#include "iri-algorithms/laser_scan_utils/entities.h"

//wolf includes
#include "constraint_corner_2D_theta.h"
#include "capture_base.h"
#include "sensor_laser_2D.h"
#include "feature_corner_2D.h"
#include "landmark_corner_2D.h"
#include "state_point.h"
#include "state_orientation.h"
#include "state_theta.h"
#include "data_association/association_tree.h"

//wolf forward declarations
//#include "feature_corner_2D.h"

//some consts.. TODO: this tuning params should be grouped in a struct and passed to the class from ros node, at constructor level 
const WolfScalar MAX_ACCEPTED_APERTURE_DIFF = 5.0*M_PI/180.; //5 degrees

class CaptureLaser2D : public CaptureBase
{
    protected:
        //constants
//         static unsigned int segment_window_size;//window size to extract segments
//         static double theta_min; //minimum theta between consecutive segments to detect corner. PI/6=0.52
//         static double theta_max_parallel; //maximum theta between consecutive segments to fuse them in a single line.
//         static double k_sigmas;//How many std_dev are tolerated to count that a point is supporting a line
//         static unsigned int max_beam_distance;//max number of beams of distance between lines to consider corner or concatenation
//         static double max_distance;//max distance between line ends to consider corner or concatenation

        //Eigen::Map<Eigen::VectorXs> ranges_; // a map to the ranges inside de data vector
        std::vector<float> ranges_; // ranges vector
        //Eigen::Map<Eigen::VectorXs> intensities_; // a map to the intensities inside the data vector
        std::vector<float> intensities_; // intensities vector
        SensorLaser2D* laser_ptr_; //specific pointer to sensor laser 2D object
        
    public:
        /** \brief Constructor with ranges
         * 
         * Constructor with ranges
         * 
         **/
        //CaptureLaser2D(const TimeStamp & _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _ranges);
        CaptureLaser2D(const TimeStamp & _ts, SensorBase* _sensor_ptr, const std::vector<float>& _ranges);

        /** \brief Constructor with ranges and intensities
         *
         * Constructor with ranges and intensities
         *
         **/
        //CaptureLaser2D(const TimeStamp & _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _ranges, const Eigen::VectorXs& _intensities);
        CaptureLaser2D(const TimeStamp & _ts, SensorBase* _sensor_ptr, const std::vector<float>& _ranges, const std::vector<float>& _intensities);

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
        virtual unsigned int extractCorners(std::list<laserscanutils::Corner> & _corner_list) const;
//         virtual unsigned int extractCorners_old(std::list<Eigen::Vector4s> & _corner_list) const;
//         void fitLine(unsigned int _idx_from, unsigned int _idx_to, const Eigen::MatrixXs& _points, Line& line_) const;

        /** \brief Extract lines from the capture
         *
         * Extract lines from the capture
         *
         **/
        unsigned int extractLines(std::list<laserscanutils::Line> & _line_list) const;

        /** \brief get corners
         *
         * Get corners
         *
         **/
        //std::list<Eigen::Vector4s> getCorners() const;
        
//         void fitLine(unsigned int _idx_from, unsigned int _idx_to, const Eigen::MatrixXs& _points, Line& line_) const;

        virtual void createFeatures(std::list<laserscanutils::Corner> & _corner_list); //TODO: should be const .... JVN: No, because new feature is added to the list

        /** \brief Create constraints given current measured features and existing landmarks
         * 
         * Create constraints given current measured features and existing landmarks.
         * Uses a Brute force Nearest Neighbor linear search
         * 
         **/        
        void establishConstraints();
        
        /** \brief Create constraints given current measured features and existing landmarks
         * 
         * Create constraints given current measured features and existing landmarks
         * Uses a Multi-Hypothesis Tree 
         * 
         **/
        void establishConstraintsMHTree();

        virtual Eigen::VectorXs computePrior() const;

        WolfScalar computeMahalanobisDistance(const FeatureBase* _feature, const LandmarkBase* _landmark);
};
#endif /* CAPTURE_LASER_2D_H_ */
