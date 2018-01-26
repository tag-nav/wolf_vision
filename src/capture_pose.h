#ifndef CAPTURE_POSE_H_
#define CAPTURE_POSE_H_

//Wolf includes
#include "capture_base.h"
#include "feature_fix.h"
#include "constraint_pose_2D.h"
#include "constraint_pose_3D.h"

//std includes
//

namespace wolf {

WOLF_PTR_TYPEDEFS(CapturePose);

//class CapturePose
class CapturePose : public CaptureBase
{
    protected:
        Eigen::VectorXs data_; ///< Raw data.
        Eigen::MatrixXs data_covariance_; ///< Noise of the capture.

    public:

        CapturePose(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);

        virtual ~CapturePose();

        virtual void emplaceFeatureAndConstraint();

};

} //namespace wolf
#endif
