#ifndef FEATURE_IMU_H_
#define FEATURE_IMU_H_

//Wolf includes
#include "feature_base.h"

//std includes


namespace wolf {

//class FeatureIMU
class FeatureIMU : public FeatureBase
{
    public:
        /** \brief Constructor from capture pointer and measure dim
         * 
         */
        FeatureIMU();

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
        FeatureIMU(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~FeatureIMU();

        /** \brief Generic interface to find constraints
         *
         * TBD
         * Generic interface to find constraints between this feature and a map (static/slam) or a previous feature
         *
         **/
        virtual void findConstraints();

};

} // namespace wolf

#endif
