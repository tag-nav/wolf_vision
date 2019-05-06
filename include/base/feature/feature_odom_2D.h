#ifndef FEATURE_ODOM_2D_H_
#define FEATURE_ODOM_2D_H_

//Wolf includes
#include "base/feature/feature_base.h"
#include "base/factor/factor_odom_2D.h"
#include "base/factor/factor_odom_2D_analytic.h"

//std includes

namespace wolf {

WOLF_PTR_TYPEDEFS(FeatureOdom2D);
    
//class FeatureOdom2D
class FeatureOdom2D : public FeatureBase
{
    public:

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
        FeatureOdom2D(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);

        virtual ~FeatureOdom2D();

        /** \brief Generic interface to find factors
         * 
         * TBD
         * Generic interface to find factors between this feature and a map (static/slam) or a previous feature
         *
         **/
        virtual void findFactors();
        
};

} // namespace wolf

#endif
