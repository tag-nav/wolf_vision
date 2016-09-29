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
class FeatureBase : public NodeBase // NodeConstrained<CaptureBase,ConstraintBase>
{
    private:
        ProblemPtr problem_ptr_;
        CaptureBasePtr capture_ptr_;
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

        /** \brief Adds a constraint from this feature (as a down node)
         */
        ConstraintBase* addConstraint(ConstraintBase* _co_ptr);

        /** \brief Removes a constraint (as a down node)
         */
        void removeConstraint(ConstraintBasePtr _co_ptr);

        /** \brief Gets the capture pointer
         */
        CaptureBase* getCapturePtr() const;
        void setCapturePtr(CaptureBase* _cap_ptr){capture_ptr_ = _cap_ptr;}

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
        
        virtual void addConstrainedBy(ConstraintBase* _ctr_ptr)
        {
            constrained_by_list_.push_back(_ctr_ptr);
        }
        virtual void removeConstrainedBy(ConstraintBase* _ctr_ptr)
        {
            constrained_by_list_.remove(_ctr_ptr);
        }
        unsigned int getHits() const
        {
            return constrained_by_list_.size();
        }
        ConstraintBaseList* getConstrainedByListPtr()
        {
            return &constrained_by_list_;
        }

        Problem* getProblem(){return problem_ptr_;}
        void setProblem(Problem* _prob_ptr){problem_ptr_ = _prob_ptr;}


};

}

// IMPLEMENTATION

#include "constraint_base.h"

namespace wolf{

inline unsigned int FeatureBase::id()
{
    return feature_id_;
}

inline void FeatureBase::removeConstraint(ConstraintBasePtr _co_ptr)
{
    constraint_list_.remove(_co_ptr);
    delete _co_ptr;
}

inline CaptureBase* FeatureBase::getCapturePtr() const
{
    return capture_ptr_;
//    return upperNodePtr();
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
