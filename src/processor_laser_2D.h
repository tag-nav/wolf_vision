/*
 * \file processor_laser_2D.h
 *
 *  Created on: Dec 15, 2015
 *      author: jsola
 */

#ifndef SRC_PROCESSOR_LASER_2D_H_
#define SRC_PROCESSOR_LASER_2D_H_

// Wolf includes
#include "processor_base.h"
#include "capture_laser_2D.h"

// TODO Shouldn't a number of these includes below be moved to the .cpp file?

//wolf includes
#include "constraint_corner_2D.h"
#include "constraint_container.h"
#include "sensor_laser_2D.h"
#include "feature_corner_2D.h"
#include "data_association/association_tree.h"

//laser_scan_utils
#include "laser_scan_utils/entities.h"
#include "laser_scan_utils/scan_basics.h"
#include "laser_scan_utils/line_detector.h"
#include "laser_scan_utils/corner_detector.h"

// Eigen includes
#include <eigen3/Eigen/Geometry>

//std includes
#include <queue>
#include <map>
#include <list>
#include <random>
#include <math.h>
#include <algorithm> //find(), sort()



//some consts.. TODO: this tuning params should be grouped in a struct and passed to the class from ros node, at constructor level
const WolfScalar MAX_ACCEPTED_APERTURE_DIFF = 20.0*M_PI/180.; //20 degrees
const WolfScalar CONTAINER_WIDTH = 2.44;
const WolfScalar CONTAINER_LENGTH = 12.20;



class ProcessorLaser2D : public ProcessorBase
{
    protected:
        SensorLaser2D* sensor_laser_ptr_; //specific pointer to sensor laser 2D object
        CaptureLaser2D* capture_laser_ptr_; // specific pointer to capture laser 2D object

    public:
        ProcessorLaser2D();
        virtual ~ProcessorLaser2D();

        void extractFeatures(CaptureBase* capture_ptr_);
        void establishConstraints(CaptureBase* capture_ptr_);

        // JOAN IS TAKING RISKS HERE. DELETE ALL FROM HERE DOWNWARDS IF YOU WANT TO MIGRATE FROM CAPTURE_LASER_2D TO HERE

    private:
        unsigned int extractCorners(std::list<laserscanutils::Corner> & _corner_list) const;
        unsigned int extractLines(std::list<laserscanutils::Line> & _line_list) const;
        void createFeatures(std::list<laserscanutils::Corner> & _corner_list) const;
        void establishConstraintsMHTree();
        void computeExpectedFeature(LandmarkBase* _landmark_ptr, Eigen::Vector4s& expected_feature_,
                                    Eigen::Matrix3s& expected_feature_cov_);
        Eigen::VectorXs computeSquaredMahalanobisDistances(const FeatureBase* _feature_ptr,
                                                           const LandmarkBase* _landmark_ptr,
                                                           const Eigen::MatrixXs& _mu);
        Eigen::VectorXs computeSquaredMahalanobisDistances(const FeatureBase* _feature_ptr,
                                                           const Eigen::Vector4s& _expected_feature,
                                                           const Eigen::Matrix3s& _expected_feature_cov,
                                                           const Eigen::MatrixXs& _mu);
        bool fitNewContainer(FeatureCorner2D* _corner_ptr, LandmarkCorner2D*& old_corner_landmark_ptr, int& feature_idx,
                             int& corner_idx);
        void createCornerLandmark(FeatureCorner2D* _corner_ptr, const Eigen::Vector3s& _feature_global_pose);
        void createContainerLandmark(FeatureCorner2D* _corner_ptr, const Eigen::Vector3s& _feature_global_pose,
                                     LandmarkCorner2D* _old_corner_landmark_ptr, int& _feature_idx, int& _corner_idx);

};
#endif /* SRC_PROCESSOR_LASER_2D_H_ */
