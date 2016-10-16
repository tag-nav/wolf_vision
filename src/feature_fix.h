#ifndef FEATURE_FIX_H_
#define FEATURE_FIX_H_


//Wolf includes
#include "feature_base.h"

//std includes
//

namespace wolf {

//class FeatureFix
class FeatureFix : public FeatureBase
{
    public:

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
        FeatureFix(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);
        virtual ~FeatureFix();
};

} // namespace wolf

#endif
