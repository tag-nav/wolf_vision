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
        ProblemWPtr problem_ptr_;
        CaptureBaseWPtr capture_ptr_;
        ConstraintBaseList constraint_list_;
        ConstraintBaseList constrained_by_list_;

        static unsigned int feature_id_count_;

    protected:
        unsigned int feature_id_;
        unsigned int track_id_; // ID of the feature track
        unsigned int landmark_id_; // ID of the landmark
        FeatureType type_id_;          ///< Feature type. See wolf.h for a list of all possible features.
        Eigen::VectorXs measurement_;                   ///<  the measurement vector
        Eigen::MatrixXs measurement_covariance_;        ///<  the measurement covariance matrix
        Eigen::MatrixXs measurement_sqrt_information_;        ///<  the squared root information matrix
        
    public:
        /** \brief Constructor from capture pointer and measure dim
         * \param _tp type of feature -- see wolf.h
         * \param _dim_measurement the dimension of the measurement space
         */
        FeatureBase(FeatureType _tp, const std::string& _type, unsigned int _dim_measurement);

        /** \brief Constructor from capture pointer and measure
         * \param _tp type of feature -- see wolf.h
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         */
        FeatureBase(FeatureType _tp, const std::string& _type, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         */
        virtual ~FeatureBase();
        void destruct();

        unsigned int id();
        unsigned int trackId(){return track_id_;}
        void setTrackId(unsigned int _tr_id){track_id_ = _tr_id;}
        unsigned int landmarkId(){return landmark_id_;}
        void setLandmarkId(unsigned int _lmk_id){landmark_id_ = _lmk_id;}
        FeatureType getTypeId(){return type_id_;}

        /** \brief Returns _ii component of measurement vector
         *
         * WARNING: To be fast, it does not check that index _ii is smaller than dimension.
         **/
        Scalar getMeasurement(unsigned int _ii) const;
        void setMeasurement(const Eigen::VectorXs& _meas);
        void setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov);
        const Eigen::MatrixXs& getMeasurementCovariance() const;
        const Eigen::MatrixXs& getMeasurementSquareRootInformation() const;


        ProblemPtr getProblem();
        void setProblem(ProblemPtr _prob_ptr);

        FrameBasePtr getFramePtr() const;

        CaptureBasePtr getCapturePtr() const;
        void setCapturePtr(CaptureBasePtr _cap_ptr){capture_ptr_ = _cap_ptr;}

        ConstraintBasePtr addConstraint(ConstraintBasePtr _co_ptr);
        void removeConstraint(ConstraintBasePtr _co_ptr);
        ConstraintBaseList* getConstraintListPtr();
        void getConstraintList(ConstraintBaseList & _ctr_list);

        const Eigen::VectorXs& getMeasurement() const;
        
        virtual void addConstrainedBy(ConstraintBasePtr _ctr_ptr);
        virtual void removeConstrainedBy(ConstraintBasePtr _ctr_ptr);
        unsigned int getHits() const;
        ConstraintBaseList* getConstrainedByListPtr();

};

}

// IMPLEMENTATION

#include "constraint_base.h"

namespace wolf{

inline void FeatureBase::addConstrainedBy(ConstraintBasePtr _ctr_ptr)
{
    constrained_by_list_.push_back(_ctr_ptr);
}

inline void FeatureBase::removeConstrainedBy(ConstraintBasePtr _ctr_ptr)
{
    constrained_by_list_.remove(_ctr_ptr);
}

inline unsigned int FeatureBase::getHits() const
{
    return constrained_by_list_.size();
}

inline ProblemPtr FeatureBase::getProblem()
{
    if (problem_ptr_ == nullptr && capture_ptr_ != nullptr)
        problem_ptr_ = capture_ptr_->getProblem();
    return problem_ptr_;
}

inline wolf::ConstraintBaseList* FeatureBase::getConstrainedByListPtr()
{
    return &constrained_by_list_;
}

inline unsigned int FeatureBase::id()
{
    return feature_id_;
}

inline void FeatureBase::removeConstraint(ConstraintBasePtr _co_ptr)
{
    constraint_list_.remove(_co_ptr);
    delete _co_ptr;
}

inline CaptureBasePtr FeatureBase::getCapturePtr() const
{
    return capture_ptr_;
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
