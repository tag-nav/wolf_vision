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

//class ConstraintBase
class ConstraintBase : public NodeLinked<FeatureBase, NodeTerminus>
{
    protected:
        ConstraintType type_; //type of constraint (types defined at wolf.h)
        const Eigen::VectorXs& measurement_; // TODO:TBD: pointer, map or copy of the feature measurement?
        const Eigen::MatrixXs& measurement_covariance_; // TODO:TBD: pointer, map or copy of the feature measurement covariance?

    public:
        /** \brief Constructor
         * 
         * Constructor
         * 
         **/
        ConstraintBase(FeatureBase* _ftr_ptr, ConstraintType _tp);

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~ConstraintBase();

        /** \brief Returns the constraint type
         * 
         * Returns the constraint type
         * 
         **/
        ConstraintType getConstraintType() const;

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
        virtual const std::vector<StateBase*> getStatePtrVector() const = 0;

        /** \brief Returns a pointer to the feature measurement
         *
         * Returns a pointer to the feature measurement
         *
         **/
        const Eigen::VectorXs& getMeasurement();

        /** \brief Returns a pointer to its capture
         *
         * Returns a pointer to its capture
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

};
#endif
