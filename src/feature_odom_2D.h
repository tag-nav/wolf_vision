#ifndef FEATURE_ODOM_2D_H_
#define FEATURE_ODOM_2D_H_

//std includes

//Wolf includes
#include "feature_base.h"
#include "constraint_odom_2D_theta.h"
#include "constraint_odom_2D_complex_angle.h"

//class FeatureOdom2D
class FeatureOdom2D : public FeatureBase
{
    public:
        /** \brief Constructor from capture pointer and measure dim
         * 
         * \param _dim_measurement the dimension of the measurement space
         * 
         */
        FeatureOdom2D(unsigned int _dim_measurement);

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
        FeatureOdom2D(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);

        virtual ~FeatureOdom2D();

        /** \brief Generic interface to find constraints
         * 
         * TBD
         * Generic interface to find constraints between this feature and a map (static/slam) or a previous feature
         *
         **/
        virtual void findConstraints();
        
};
#endif
