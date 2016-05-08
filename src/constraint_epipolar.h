#ifndef CONSTRAINT_EPIPOLAR_H
#define CONSTRAINT_EPIPOLAR_H

#include "constraint_base.h"

namespace wolf {

class ConstraintEpipolar : public ConstraintBase
{
    public:
        ConstraintEpipolar(FeatureBase* _feature_ptr, FeatureBase* _feature_other_ptr);

        virtual ~ConstraintEpipolar();

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const{return JAC_ANALYTIC;}

        /** \brief Returns a vector of scalar pointers to the first element of all state blocks involved in the constraint
         **/
        virtual const std::vector<Scalar*> getStateBlockPtrVector(){return std::vector<Scalar*>(0);}

        /** \brief Returns a vector of pointers to the states in which this constraint depends
         **/
        virtual const std::vector<StateBlock*> getStatePtrVector() const{return std::vector<StateBlock*>(0);}

        /** \brief Returns the constraint residual size
         **/
        virtual unsigned int getSize() const{return 0;}
};

inline ConstraintEpipolar::ConstraintEpipolar(FeatureBase* _feature_ptr, FeatureBase* _feature_other_ptr) :
        ConstraintBase(CTR_EPIPOLAR, _feature_other_ptr, CTR_INACTIVE)
{
    setType("EPIPOLAR");
}

inline ConstraintEpipolar::~ConstraintEpipolar(){}

} // namespace wolf

#endif // CONSTRAINT_EPIPOLAR_H
