#ifndef FEATURE_GPS_FIX_H_
#define FEATURE_GPS_FIX_H_

//Wolf includes
#include <constraint_GPS_2D.h>
#include "feature_base.h"

//std includes

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FeatureGPSFix);

//class FeatureGPSFix
class FeatureGPSFix : public FeatureBase
{
    public:

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
        FeatureGPSFix(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);
        virtual ~FeatureGPSFix();
};

} // namespace wolf

#endif
