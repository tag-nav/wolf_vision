#ifndef FEATURE_BASE_H_
#define FEATURE_BASE_H_

// Forward declarations for node templates
namespace wolf{
class CaptureBase;
class ConstraintBase;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"

//std includes

namespace wolf {


//class FeatureBase
class FeatureBase : public NodeBase, public std::enable_shared_from_this<FeatureBase>
{
    private:
        CaptureBaseWPtr capture_ptr_;
        ConstraintBaseList constraint_list_;
        ConstraintBaseList constrained_by_list_;

        static unsigned int feature_id_count_;
        bool is_removing_; ///< A flag for safely removing nodes from the Wolf tree. See remove().

    protected:
        unsigned int feature_id_;
        unsigned int track_id_; // ID of the feature track
        unsigned int landmark_id_; // ID of the landmark
        Eigen::VectorXs measurement_;                   ///<  the measurement vector
        Eigen::MatrixXs measurement_covariance_;        ///<  the measurement covariance matrix
        Eigen::MatrixXs measurement_sqrt_information_transposed_;        ///<  the squared root information matrix
        Eigen::VectorXs expectation_;                   ///<  expectation
        
    public:
        /** \brief Constructor from capture pointer and measure dim
         * \param _tp type of feature -- see wolf.h
         * \param _dim_measurement the dimension of the measurement space
         */
        FeatureBase(const std::string& _type, unsigned int _dim_measurement);

        /** \brief Constructor from capture pointer and measure
         * \param _tp type of feature -- see wolf.h
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         */
        FeatureBase(const std::string& _type, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);

        virtual ~FeatureBase();
        void remove();

        // properties
        unsigned int id();
        unsigned int trackId(){return track_id_;}
        void setTrackId(unsigned int _tr_id){track_id_ = _tr_id;}
        unsigned int landmarkId(){return landmark_id_;}
        void setLandmarkId(unsigned int _lmk_id){landmark_id_ = _lmk_id;}

        // values
        /* \brief Returns _ii component of measurement vector
         *
         * WARNING: To be fast, it does not check that index _ii is smaller than dimension.
         */
        Scalar getMeasurement(unsigned int _ii) const;
        const Eigen::VectorXs& getMeasurement() const;
        void setMeasurement(const Eigen::VectorXs& _meas);
        void setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov);
        const Eigen::MatrixXs& getMeasurementCovariance() const;
        const Eigen::MatrixXs& getMeasurementSquareRootInformationTransposed() const;

        const Eigen::VectorXs& getExpectation() const;
        void setExpectation(const Eigen::VectorXs& expectation);

        // wolf tree access
        ProblemPtr getProblem();

        FrameBasePtr getFramePtr() const;

        CaptureBasePtr getCapturePtr() const;
        void setCapturePtr(CaptureBasePtr _cap_ptr){capture_ptr_ = _cap_ptr;}

        ConstraintBasePtr addConstraint(ConstraintBasePtr _co_ptr);
        ConstraintBaseList& getConstraintList();

        virtual ConstraintBasePtr addConstrainedBy(ConstraintBasePtr _ctr_ptr);
        unsigned int getHits() const;
        ConstraintBaseList& getConstrainedByList();

        // all constraints
        void getConstraintList(ConstraintBaseList & _ctr_list);

};



}

// IMPLEMENTATION

#include "constraint_base.h"

namespace wolf{

inline ConstraintBasePtr FeatureBase::addConstrainedBy(ConstraintBasePtr _ctr_ptr)
{
    constrained_by_list_.push_back(_ctr_ptr);
    _ctr_ptr->setFeatureOtherPtr( shared_from_this() );
    return _ctr_ptr;
}

inline unsigned int FeatureBase::getHits() const
{
    return constrained_by_list_.size();
}

inline ProblemPtr FeatureBase::getProblem()
{
    ProblemPtr prb = problem_ptr_.lock();
    if (!prb)
    {
        CaptureBasePtr cap = capture_ptr_.lock();
        if (cap)
        {
            prb = cap->getProblem();
            problem_ptr_ = prb;
        }
    }
    return prb;
}

inline wolf::ConstraintBaseList& FeatureBase::getConstrainedByList()
{
    return constrained_by_list_;
}

inline unsigned int FeatureBase::id()
{
    return feature_id_;
}

inline CaptureBasePtr FeatureBase::getCapturePtr() const
{
    return capture_ptr_.lock();
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

inline const Eigen::MatrixXs& FeatureBase::getMeasurementSquareRootInformationTransposed() const
{
    return measurement_sqrt_information_transposed_;
}

inline void FeatureBase::setMeasurement(const Eigen::VectorXs& _meas)
{
    measurement_ = _meas;
}

inline const Eigen::VectorXs& FeatureBase::getExpectation() const
{
    return expectation_;
}

inline void FeatureBase::setExpectation(const Eigen::VectorXs& expectation)
{
    expectation_ = expectation;
}

} // namespace wolf

#endif
