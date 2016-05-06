/*
 * \file processor_laser_2D.h
 *
 *  Created on: Dec 15, 2015
 *      author: jsola
 */

#ifndef SRC_PROCESSOR_LASER_2D_H_
#define SRC_PROCESSOR_LASER_2D_H_


#include "processor_base.h"


//TODO try this, and remove includes below
//class SensorLaser2D;
//class CaptureLaser2D;
//class LandmarkBase;
//class FeatureCorner2D;
//class ConstraintCorner2D;
//class ConstraintContainer;


// TODO Shouldn't a number of these includes below be moved to the .cpp file?

//wolf includes
#include "sensor_laser_2D.h"
#include "capture_laser_2D.h"
#include "feature_corner_2D.h"
#include "constraint_corner_2D.h"
#include "constraint_container.h"

#include "data_association/association_tree.h"

//std includes
#include <list>



namespace wolf {


//some consts.. TODO: this tuning params should be grouped in a struct and passed to the class from ros node, at constructor level
const Scalar MAX_ACCEPTED_APERTURE_DIFF = 20.0*M_PI/180.; //20 degrees
const Scalar CONTAINER_WIDTH = 2.44;
const Scalar CONTAINER_LENGTH = 12.20;

struct ProcessorParamsLaser2D : public ProcessorParamsBase
{
        Scalar max_accepted_aperture_diff = 20.0*M_PI/180.; //20 degrees
        Scalar container_width = 2.44;
        Scalar container_length = 12.20;
};



class ProcessorLaser2D : public ProcessorBase
{
    protected:
        SensorLaser2D* sensor_laser_ptr_; //specific pointer to sensor laser 2D object
        CaptureLaser2D* capture_laser_ptr_; // specific pointer to capture laser 2D object

    public:
        ProcessorLaser2D();
        virtual ~ProcessorLaser2D();

        void process(CaptureBase *_capture_ptr);

        virtual bool voteForKeyFrame();
        virtual bool keyFrameCallback(FrameBase* _keyframe_ptr){return false;};
        virtual void init(CaptureBase* _origin_ptr);

    protected:
//        virtual void preProcess(){}
//        virtual void postProcess(){}

        // JS: These two fcns can be removed and substituted by process() above.
    private:
        void extractFeatures(CaptureBase* _capture_ptr);
        void establishConstraints(CaptureBase* _capture_ptr_);

        // JOAN_S IS TAKING RISKS HERE. DELETE ALL FROM HERE DOWNWARDS IF YOU WANT TO MIGRATE FROM CAPTURE_LASER_2D TO HERE

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

    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};

inline bool ProcessorLaser2D::voteForKeyFrame()
{
    return false;
}

inline void ProcessorLaser2D::init(CaptureBase* _origin_ptr)
{
}

} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
//const bool registered_prc_laser_2d = ProcessorFactory::get()->registerCreator("LASER 2D", ProcessorLaser2D::create);
}
} // namespace wolf


#endif /* SRC_PROCESSOR_LASER_2D_H_ */
