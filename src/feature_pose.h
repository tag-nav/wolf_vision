#ifndef FEATURE_POSE_H_
#define FEATURE_POSE_H_


//Wolf includes
#include "feature_base.h"

//std includes
//

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FeaturePose);

//class FeaturePose
class FeaturePose : public FeatureBase
{
    public:

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
        FeaturePose(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);
        virtual ~FeaturePose();
};

} // namespace wolf

#endif
