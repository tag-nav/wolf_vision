#ifndef CONSTRAINT_BASE_H_
#define CONSTRAINT_BASE_H_

// Forward declarations for node templates
namespace wolf{
class FeatureBase;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"

//std includes

namespace wolf {


//class ConstraintBase
class ConstraintBase : public NodeBase, public std::enable_shared_from_this<ConstraintBase>
{
    private:
        FeatureBaseWPtr feature_ptr_;                    ///< FeatureBase pointer (upper node)

        static unsigned int constraint_id_count_;

    protected:
        unsigned int constraint_id_;
        ConstraintType type_id_;                        ///< type of constraint (types defined at wolf.h)
        ConstraintCategory category_;                   ///< category of constraint (types defined at wolf.h)
        ConstraintStatus status_;                       ///< status of constraint (types defined at wolf.h)
        bool apply_loss_function_;                      ///< flag for applying loss function to this constraint
        FrameBaseWPtr frame_other_ptr_;                    ///< FrameBase pointer (for category CTR_FRAME)
        FeatureBaseWPtr feature_other_ptr_;                ///< FeatureBase pointer (for category CTR_FEATURE)
        LandmarkBaseWPtr landmark_other_ptr_;              ///< LandmarkBase pointer (for category CTR_LANDMARK)

    public:

        /** \brief Constructor of category CTR_ABSOLUTE
         **/
        ConstraintBase(ConstraintType _tp, bool _apply_loss_function, ConstraintStatus _status);

        /** \brief Constructor of category CTR_FRAME
         **/
        ConstraintBase(ConstraintType _tp, FrameBasePtr _frame_ptr, bool _apply_loss_function, ConstraintStatus _status);

        /** \brief Constructor of category CTR_FEATURE
         **/
        ConstraintBase(ConstraintType _tp, FeatureBasePtr _feature_ptr, bool _apply_loss_function, ConstraintStatus _status);

        /** \brief Constructor of category CTR_LANDMARK
         **/
        ConstraintBase(ConstraintType _tp, LandmarkBasePtr _landmark_ptr, bool _apply_loss_function, ConstraintStatus _status);

        virtual ~ConstraintBase();
        void remove();

        unsigned int id();

        /** \brief Returns the constraint type
         **/
        ConstraintType getTypeId() const;

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const = 0;

        /** \brief Returns a vector of scalar pointers to the first element of all state blocks involved in the constraint
         **/
        virtual const std::vector<Scalar*> getStateBlockPtrVector() = 0;

        /** \brief Returns a vector of pointers to the states in which this constraint depends
         **/
        virtual const std::vector<StateBlock*> getStatePtrVector() const = 0;

        /** \brief Returns a reference to the feature measurement
         **/
        virtual const Eigen::VectorXs& getMeasurement() const;

        /** \brief Returns a reference to the feature measurement covariance
         **/
        virtual const Eigen::MatrixXs& getMeasurementCovariance() const;

        /** \brief Returns a reference to the feature measurement square root information
         **/
        virtual const Eigen::MatrixXs& getMeasurementSquareRootInformation() const;

        /** \brief Returns a pointer to the feature constrained from
         **/
        FeatureBasePtr getFeaturePtr() const;
        void setFeaturePtr(const FeatureBasePtr _ft_ptr){feature_ptr_ = _ft_ptr;}

        /** \brief Returns a pointer to its capture
         **/
        CaptureBasePtr getCapturePtr() const;

        /** \brief Returns the constraint residual size
         **/
        virtual unsigned int getSize() const = 0;

        /** \brief Gets the category
         */
        ConstraintCategory getCategory() const;

        /** \brief Gets the status
         */
        ConstraintStatus getStatus() const;

        /** \brief Sets the status
         */
        void setStatus(ConstraintStatus _status);

        /** \brief Gets the apply loss function flag
         */
        bool getApplyLossFunction();

        /** \brief Gets the apply loss function flag
         */
        void setApplyLossFunction(const bool _apply);

        /** \brief Returns a pointer to the frame constrained to
         **/
        FrameBasePtr getFrameOtherPtr();

        /** \brief Returns a pointer to the feature constrained to
         **/
        FeatureBasePtr getFeatureOtherPtr();

        /** \brief Returns a pointer to the landmark constrained to
         **/
        LandmarkBasePtr getLandmarkOtherPtr();

        ProblemPtr getProblem();

};


}

// IMPLEMENTATION //

#include "problem.h"
#include "frame_base.h"
#include "feature_base.h"
#include "sensor_base.h"
#include "landmark_base.h"

namespace wolf{

inline void ConstraintBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        std::cout << "Removing         c" << id() << std::endl;
        //                shared_ptr<c> this_c = shared_from_this();  // keep this alive while removing it
        feature_ptr_->getConstraintListPtr()->remove(this);          // remove from upstream
        if (feature_ptr_->getConstraintListPtr()->empty() && feature_ptr_->getConstrainedByListPtr()->empty())
            feature_ptr_->remove();                   // remove upstream

        // add constraint to be removed from solver
        if (getProblem() != nullptr)
            getProblem()->removeConstraintPtr(this);


        // remove other: {Frame, feature, Landmark}
        switch (category_)
        {
            case CTR_FRAME:
                frame_other_ptr_->getConstrainedByListPtr()->remove(this);
                if (frame_other_ptr_->getConstrainedByListPtr()->empty() && frame_other_ptr_->getCaptureListPtr()->empty())
                    frame_other_ptr_->remove();
                break;
            case CTR_FEATURE:
                feature_other_ptr_->getConstrainedByListPtr()->remove(this);
                if (feature_other_ptr_->getConstrainedByListPtr()->empty() && feature_other_ptr_->getConstraintListPtr()->empty())
                    feature_other_ptr_->remove();
                break;
            case CTR_LANDMARK:
                landmark_other_ptr_->getConstrainedByListPtr()->remove(this);
                if (landmark_other_ptr_->getConstrainedByListPtr()->empty())
                    landmark_other_ptr_->remove();
                break;
            case CTR_ABSOLUTE:
                break;
            default:
                break;
        }
    }

}

inline wolf::ProblemPtr ConstraintBase::getProblem()
{
    if (problem_ptr_ == nullptr && feature_ptr_ != nullptr)
        problem_ptr_ = feature_ptr_->getProblem();
    return problem_ptr_;
}

inline unsigned int ConstraintBase::id()
{
    return constraint_id_;
}

inline ConstraintType ConstraintBase::getTypeId() const
{
    return type_id_;
}

inline FeatureBasePtr ConstraintBase::getFeaturePtr() const
{
    return feature_ptr_;
}

inline ConstraintCategory ConstraintBase::getCategory() const
{
    return category_;
}

inline ConstraintStatus ConstraintBase::getStatus() const
{
    return status_;
}

inline bool ConstraintBase::getApplyLossFunction()
{
    return apply_loss_function_;
}

inline void ConstraintBase::setApplyLossFunction(const bool _apply)
{
    if (apply_loss_function_ != _apply)
    {
        if (getProblem() == nullptr)
            std::cout << "constraint not linked with Problem, apply loss function change not notified" << std::endl;
        else
        {
            getProblem()->removeConstraintPtr(this);
            getProblem()->addConstraintPtr(this);
        }
    }
}

inline FrameBasePtr ConstraintBase::getFrameOtherPtr()
{
    return frame_other_ptr_;
}

inline FeatureBasePtr ConstraintBase::getFeatureOtherPtr()
{
    return feature_other_ptr_;
}

inline LandmarkBasePtr ConstraintBase::getLandmarkOtherPtr()
{
    //    return landmark_other_ptr_;// TODO remove line
    //    return landmark_other_ptr_.lock();// TODO uncomment line
    return std::shared_ptr<LandmarkBase>(landmark_other_ptr_);// TODO remove line
}

} // namespace wolf
#endif
