#ifndef CONSTRAINT_BASE_H_
#define CONSTRAINT_BASE_H_

// Forward declarations for node templates
namespace wolf{
class FeatureBase;
class NodeTerminus;
}

//Wolf includes
#include "wolf.h"
#include "node_linked.h"

//std includes
//

namespace wolf {

//TODO: add a member to indicate how jacobian is computed, called "jacobian_method_"
//class ConstraintBase
class ConstraintBase : public NodeLinked<FeatureBase, NodeTerminus>
{
    private:
        static unsigned int constraint_id_count_;
    protected:
        unsigned int constraint_id_;
        ConstraintType type_id_;                        ///< type of constraint (types defined at wolf.h)
        ConstraintCategory category_;                   ///< category of constraint (types defined at wolf.h)
        ConstraintStatus status_;                       ///< status of constraint (types defined at wolf.h)
        bool apply_loss_function_;                      ///< flag for applying loss function to this constraint
        FrameBase* frame_ptr_;                          ///< FrameBase pointer (for category CTR_FRAME)
        FeatureBase* feature_ptr_;                      ///< FeatureBase pointer (for category CTR_FEATURE)
        LandmarkBase* landmark_ptr_;                    ///< LandmarkBase pointer (for category CTR_LANDMARK)

    public:

        /** \brief Constructor of category CTR_ABSOLUTE
         **/
        ConstraintBase(ConstraintType _tp, bool _apply_loss_function, ConstraintStatus _status);

        /** \brief Constructor of category CTR_FRAME
         **/
        ConstraintBase(ConstraintType _tp, FrameBase* _frame_ptr, bool _apply_loss_function, ConstraintStatus _status);

        /** \brief Constructor of category CTR_FEATURE
         **/
        ConstraintBase(ConstraintType _tp, FeatureBase* _feature_ptr, bool _apply_loss_function, ConstraintStatus _status);

        /** \brief Constructor of category CTR_LANDMARK
         **/
        ConstraintBase(ConstraintType _tp, LandmarkBase* _landmark_ptr, bool _apply_loss_function, ConstraintStatus _status);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/
        virtual ~ConstraintBase();

        unsigned int id();

        /** \brief Returns the constraint type
         **/
        ConstraintType getType() const;

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
        const Eigen::VectorXs& getMeasurement() const;

        /** \brief Returns a reference to the feature measurement covariance
         **/
        const Eigen::MatrixXs& getMeasurementCovariance() const;

        /** \brief Returns a reference to the feature measurement square root information
         **/
        const Eigen::MatrixXs& getMeasurementSquareRootInformation() const;

        /** \brief Returns a pointer to the feature constrained from
         **/
        FeatureBase* getFeaturePtr() const;

        /** \brief Returns a pointer to its capture
         **/
        CaptureBase* getCapturePtr() const;

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
        FrameBase* getFrameOtherPtr();

        /** \brief Returns a pointer to the feature constrained to
         **/
        FeatureBase* getFeatureOtherPtr();

        /** \brief Returns a pointer to the landmark constrained to
         **/
        LandmarkBase* getLandmarkOtherPtr();
};

inline unsigned int ConstraintBase::id()
{
    return constraint_id_;
}

// IMPLEMENTATION //

inline ConstraintType ConstraintBase::getType() const
{
    return type_id_;
}

inline FeatureBase* ConstraintBase::getFeaturePtr() const
{
    return upperNodePtr();
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

inline FrameBase* ConstraintBase::getFrameOtherPtr()
{
    return frame_ptr_;
}

inline FeatureBase* ConstraintBase::getFeatureOtherPtr()
{
    return feature_ptr_;
}

inline LandmarkBase* ConstraintBase::getLandmarkOtherPtr()
{
    return landmark_ptr_;
}

} // namespace wolf
#endif
