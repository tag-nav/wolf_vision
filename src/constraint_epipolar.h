#ifndef CONSTRAINT_EPIPOLAR_H
#define CONSTRAINT_EPIPOLAR_H

#include "constraint_base.h"

namespace wolf {

class ConstraintEpipolar : public ConstraintBase
{
    public:
        ConstraintEpipolar();

        virtual ~ConstraintEpipolar();


        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const{return JAC_ANALYTIC;}

        /** \brief Returns a vector of scalar pointers to the first element of all state blocks involved in the constraint
         **/
        virtual const std::vector<WolfScalar*> getStateBlockPtrVector(){return std::vector<WolfScalar*>(0);}

        /** \brief Returns a vector of pointers to the states in which this constraint depends
         **/
        virtual const std::vector<StateBlock*> getStatePtrVector() const{return std::vector<StateBlock*>(0);}

        /** \brief Returns the constraint residual size
         **/
        virtual unsigned int getSize() const{return 0;}
};

inline ConstraintEpipolar::ConstraintEpipolar():ConstraintBase(CTR_EPIPOLAR,CTR_INACTIVE)
{
}

inline ConstraintEpipolar::~ConstraintEpipolar(){}

} // namespace wolf

#endif // CONSTRAINT_EPIPOLAR_H
