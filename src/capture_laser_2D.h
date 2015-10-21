
#ifndef CAPTURE_LASER_2D_H_
#define CAPTURE_LASER_2D_H_

//std includes
#include <queue>
#include <map>
#include <random>
#include <math.h>
#include <algorithm> //find(), sort()

// Eigen ingludes
#include <eigen3/Eigen/Geometry>

//laser_scan_utils
#include "laser_scan_utils/entities.h"
#include "laser_scan_utils/scan_basics.h"
#include "laser_scan_utils/line_detector.h"
#include "laser_scan_utils/corner_detector.h"

//wolf includes
#include "constraint_corner_2D_theta.h"
#include "constraint_container.h"
#include "capture_base.h"
#include "sensor_laser_2D.h"
#include "feature_corner_2D.h"
#include "landmark_corner_2D.h"
#include "landmark_container.h"
#include "state_point.h"
#include "state_orientation.h"
#include "state_theta.h"
#include "data_association/association_tree.h"

//wolf forward declarations
//#include "feature_corner_2D.h"

//some consts.. TODO: this tuning params should be grouped in a struct and passed to the class from ros node, at constructor level 
const WolfScalar MAX_ACCEPTED_APERTURE_DIFF = 20.0*M_PI/180.; //20 degrees
const WolfScalar CONTAINER_WIDTH = 2.44;
const WolfScalar CONTAINER_LENGTH = 12.20;

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
        std::vector<float> ranges_; // ranges vector. Type float to match ROS LaserScan message 
        //Eigen::Map<Eigen::VectorXs> intensities_; // a map to the intensities inside the data vector
        std::vector<float> intensities_; // intensities vector. Type float to match ROS LaserScan message 
        SensorLaser2D* laser_ptr_; //specific pointer to sensor laser 2D object
        
    public:
        /** \brief Constructor with ranges
         * 
         * Constructor with ranges
         * 
         **/
        CaptureLaser2D(const TimeStamp & _ts, SensorBase* _sensor_ptr, const std::vector<float>& _ranges);

        /** \brief Constructor with ranges and intensities
         *
         * Constructor with ranges and intensities
         *
         **/
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
         * Create constraints given current measured features and existing landmarks
         * Uses a Multi-Hypothesis Tree 
         * 
         **/
        void establishConstraintsMHTree();

        void computeExpectedFeature(LandmarkBase* _landmark_ptr, Eigen::Vector4s& expected_feature_, Eigen::Matrix3s& expected_feature_cov_);

        virtual Eigen::VectorXs computePrior(const TimeStamp& _now) const;

        Eigen::VectorXs computeSquaredMahalanobisDistances(const FeatureBase* _feature_ptr, const LandmarkBase* _landmark_ptr, const Eigen::MatrixXs& _mu);
        Eigen::VectorXs computeSquaredMahalanobisDistances(const FeatureBase* _feature_ptr, const Eigen::Vector4s& _expected_feature, const Eigen::Matrix3s& _expected_feature_cov, const Eigen::MatrixXs& _mu);

        /** \brief Tries to fit a container given a feature and all existing landmarks
         *
         * Tries to fit a container given a feature and all existing landmarks. Returns false if not fitted and true otherwise.
         * Returns via params feature_idx & corner_idx the index of the feature and the landmark respectively and in old_corner_landmark_ptr, the corresponding landmark:
         *
         *    1 ------------------------- 0
         *    |                           |
         *    |                           |
         *    |                           |
         *    2 ------------------------- 3
         *
         **/
        bool fitNewContainer(FeatureCorner2D* _corner_ptr, LandmarkCorner2D*& old_corner_landmark_ptr, int& feature_idx, int& corner_idx);

        void createCornerLandmark(FeatureCorner2D* _corner_ptr, const Eigen::Vector3s& _feature_global_pose);

        void createContainerLandmark(FeatureCorner2D* _corner_ptr, const Eigen::Vector3s& _feature_global_pose, LandmarkCorner2D* _old_corner_landmark_ptr, int& _feature_idx, int& _corner_idx);
};
#endif /* CAPTURE_LASER_2D_H_ */
