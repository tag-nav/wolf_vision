#ifndef FEATURE_BASE_H_
#define FEATURE_BASE_H_

// Forward declarations for node templates
namespace wolf{
class CaptureBase;
class ConstraintBase;
}

//Wolf includes
#include "wolf.h"
#include "node_linked.h"
#include "node_constrained.h"

//std includes


namespace wolf {

//class FeatureBase
class FeatureBase : public NodeConstrained<CaptureBase,ConstraintBase>
{
    protected:
        FeatureType type_;          ///< Feature type. See wolf.h for a list of all possible features.
        Eigen::VectorXs measurement_;                   ///<  the measurement vector
        Eigen::MatrixXs measurement_covariance_;        ///<  the measurement covariance matrix
        Eigen::MatrixXs measurement_sqrt_information_;        ///<  the squared root information matrix
        
    public:
        /** \brief Constructor from capture pointer and measure dim
         * 
         * \param _dim_measurement the dimension of the measurement space
         */
        FeatureBase(FeatureType _tp, unsigned int _dim_measurement);

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         */
        FeatureBase(FeatureType _tp, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         */
        virtual ~FeatureBase();

        /** \brief Adds a constraint from this feature (as a down node)
         */
        void addConstraint(ConstraintBase* _co_ptr);

        /** \brief Gets the capture pointer
         */
        CaptureBase* getCapturePtr() const;

        /** \brief Gets the frame pointer
         */
        FrameBase* getFramePtr() const;

        /** \brief Gets the constraint list (down nodes) pointer
         */
        ConstraintBaseList* getConstraintListPtr();
        
        void getConstraintList(ConstraintBaseList & _ctr_list);

        const Eigen::VectorXs& getMeasurement() const;
        
        /** \brief Returns _ii component of measurement vector
         * 
         * WARNING: To be fast, it does not check that index _ii is smaller than dimension.
         **/
        Scalar getMeasurement(unsigned int _ii) const;

        /** \brief Returns a reference to the feature measurement covariance
         **/
        const Eigen::MatrixXs& getMeasurementCovariance() const;

        /** \brief Returns a reference to the feature measurement square root information
         **/
        const Eigen::MatrixXs& getMeasurementSquareRootInformation() const;

        void setMeasurement(const Eigen::VectorXs& _meas);
        
        void setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov);
        
};

inline CaptureBase* FeatureBase::getCapturePtr() const
{
    return upperNodePtr();
}

inline Scalar FeatureBase::getMeasurement(unsigned int _ii) const
{
    return measurement_(_ii);
}

inline const Eigen::VectorXs& FeatureBase::getMeasurement() const
{
    return measurement_;
}

inline const Eigen::MatrixXs& FeatureBase::getMeasurementCovariance() const
{
    return measurement_covariance_;
}

inline const Eigen::MatrixXs& FeatureBase::getMeasurementSquareRootInformation() const
{
    return measurement_sqrt_information_;
}

inline void FeatureBase::setMeasurement(const Eigen::VectorXs& _meas)
{
    measurement_ = _meas;
}

} // namespace wolf

#endif
