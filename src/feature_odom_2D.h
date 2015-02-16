#ifndef FEATURE_ODOM_2D_H_
#define FEATURE_ODOM_2D_H_

//std includes

//Wolf includes
#include "feature_base.h"
#include "correspondence_odom_2D_theta.h"
#include "correspondence_odom_2D_complex_angle.h"

//class FeatureOdom2D
class FeatureOdom2D : public FeatureBase
{
    public:
        /** \brief Constructor from capture pointer and measure dim
         * 
         * \param _capt_ptr a shared pointer to the Capture up node
         * \param _dim_measurement the dimension of the measurement space
         * 
         */
		FeatureOdom2D(const CaptureBasePtr& _capt_ptr, unsigned int _dim_measurement);

        /** \brief Constructor from capture pointer and measure
         *
         * \param _capt_ptr a shared pointer to the Capture up node
         * \param _measurement the measurement
         *
         */
		//FeatureOdom2D(const CaptureBasePtr& _capt_ptr, const Eigen::VectorXs& _measurement);

        /** \brief Constructor from capture pointer and measure
         *
         * \param _capt_ptr a shared pointer to the Capture up node
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
		FeatureOdom2D(const CaptureBasePtr& _capt_ptr, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);

        virtual ~FeatureOdom2D();

        /** \brief Generic interface to find correspondences
         * 
         * TBD
         * Generic interface to find correspondences between this feature and a map (static/slam) or a previous feature
         *
         **/
        virtual void findCorrespondences();
        
};
#endif
