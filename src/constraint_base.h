#ifndef CONSTRAINT_BASE_H_
#define CONSTRAINT_BASE_H_

// Forward declarations for node templates
class FeatureBase;
class NodeTerminus;

//std includes
//

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "node_linked.h"
#include "feature_base.h"
#include "node_terminus.h"

//TODO: add a member to indicate how jacobian is computed, called "jacobian_method_"
//class ConstraintBase
class ConstraintBase : public NodeLinked<FeatureBase, NodeTerminus>
{
    protected:
        ConstraintType type_;                           ///< type of constraint (types defined at wolf.h)
        ConstraintCategory category_;                   ///< category of constraint (types defined at wolf.h)
        ConstraintStatus status_;                       ///< status of constraint (types defined at wolf.h)
        FrameBase* frame_ptr_;                          ///< FrameBase pointer (for category CTR_FRAME)
        FeatureBase* feature_ptr_;                      ///< FeatureBase pointer (for category CTR_FEATURE)
        LandmarkBase* landmark_ptr_;                    ///< LandmarkBase pointer (for category CTR_LANDMARK)

    public:
        /** \brief Constructor of category CTR_ABSOLUTE
         *
         * Constructor of category CTR_ABSOLUTE
         *
         **/
        ConstraintBase(ConstraintType _tp, ConstraintStatus _status);

        /** \brief Constructor of category CTR_FRAME
         *
         * Constructor of category CTR_FRAME
         *
         **/
        ConstraintBase(ConstraintType _tp, FrameBase* _frame_ptr, ConstraintStatus _status);

        /** \brief Constructor of category CTR_FEATURE
         *
         * Constructor of category CTR_FEATURE
         *
         **/
        ConstraintBase(ConstraintType _tp, FeatureBase* _feature_ptr, ConstraintStatus _status);

        /** \brief Constructor of category CTR_LANDMARK
         * 
         * Constructor of category CTR_LANDMARK
         * 
         **/
        ConstraintBase(ConstraintType _tp, LandmarkBase* _landmark_ptr, ConstraintStatus _status);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/
        virtual ~ConstraintBase();

        /** \brief Returns the constraint type
         * 
         * Returns the constraint type
         * 
         **/
        ConstraintType getType() const;

        /** \brief Returns a vector of scalar pointers to the first element of all state blocks involved in the constraint
         *
         * Returns a vector of scalar pointers to the first element of all state blocks involved in the constraint.
         *
         **/
        virtual const std::vector<WolfScalar*> getStateBlockPtrVector() = 0;

        /** \brief Returns a vector of pointers to the states
         *
         * Returns a vector of pointers to the state in which this constraint depends
         *
         **/
        virtual const std::vector<StateBlock*> getStatePtrVector() const = 0;

        /** \brief Returns a pointer to the feature measurement
         *
         * Returns a pointer to the feature measurement
         *
         **/
        const Eigen::VectorXs& getMeasurement() const;

        /** \brief Returns a pointer to the feature measurement covariance
         *
         * Returns a pointer to the feature measurement covariance
         *
         **/
        const Eigen::MatrixXs& getMeasurementCovariance() const;

        /** \brief Returns a pointer to the feature constrained from
         *
         * Returns a pointer to the feature constrained from
         *
         **/
        FeatureBase* getFeaturePtr() const;

        /** \brief Returns a pointer to its capture
         *
         * Returns a pointer to its capture
         *
         **/
        CaptureBase* getCapturePtr() const;

        /** \brief Returns the constraint residual size
         *
         * Returns the constraint residual size
         *
         **/
        virtual unsigned int getSize() const = 0;

        /** \brief Gets the category
         *
         * Gets the category
         *
         */
        ConstraintCategory getCategory() const;

        /** \brief Gets the status
         *
         * Gets the status
		 *
         */
        ConstraintStatus getStatus() const;

        /** \brief Sets the status
         *
         * Sets the status
		 *
         */
        void setStatus(ConstraintStatus _status);

        /** \brief Returns a pointer to the frame constrained to
         *
         * Returns a pointer to the frame constrained to
         *
         **/
        FrameBase* getFrameOtherPtr();

        /** \brief Returns a pointer to the feature constrained to
         *
         * Returns a pointer to the feature constrained to
         *
         **/
        FeatureBase* getFeatureOtherPtr();

        /** \brief Returns a pointer to the landmark constrained to
         *
         * Returns a pointer to the landmark constrained to
         *
         **/
        LandmarkBase* getLandmarkOtherPtr();
};
#endif
