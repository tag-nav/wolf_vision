#ifndef FEATURE_BASE_H_
#define FEATURE_BASE_H_

// Forward declarations for node templates
class CaptureBase;
class ConstraintBase;

//std includes

//Wolf includes
#include "wolf.h"
#include "node_linked.h"
#include "capture_base.h"
#include "constraint_base.h"

//class FeatureBase
class FeatureBase : public NodeLinked<CaptureBase,ConstraintBase>
{
    protected:
        Eigen::VectorXs measurement_;
        Eigen::MatrixXs measurement_covariance_; ///< Noise of the measurement
        
    public:
        /** \brief Constructor from capture pointer and measure dim
         * 
         * \param _dim_measurement the dimension of the measurement space
         * 
         */
        FeatureBase(unsigned int _dim_measurement);

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
        FeatureBase(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);

        virtual ~FeatureBase();
        
        void addConstraint(ConstraintBase* _co_ptr);

        CaptureBase* getCapturePtr() const;

        FrameBase* getFramePtr() const;
        
        ConstraintBaseList* getConstraintListPtr();
        
        void getConstraintList(ConstraintBaseList & _ctr_list);

//        Eigen::VectorXs * getMeasurementPtr();
//
//        Eigen::MatrixXs * getMeasurementCovariancePtr();

        const Eigen::VectorXs & getMeasurement() const;
        
        /** \brief Returns _ii component of measurement vector
         * 
         * Returns _ii component of measurement_ vector
         * WARNING: To be fast, it does not check that index _ii is smaller than dimension.
         * 
         **/
        WolfScalar getMeasurement(unsigned int _ii) const;

        const Eigen::MatrixXs & getMeasurementCovariance() const;

        void setMeasurement(const Eigen::VectorXs & _meas);
        
        void setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov);
        
        /** \brief prints object's info
         * 
         * prints object's info
         * 
         **/
        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
        
};
#endif
