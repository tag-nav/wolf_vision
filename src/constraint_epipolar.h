#ifndef CONSTRAINT_EPIPOLAR_H
#define CONSTRAINT_EPIPOLAR_H

#include "constraint_base.h"

namespace wolf {

class ConstraintEpipolar : public ConstraintBase
{
    public:
        ConstraintEpipolar(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE);

        virtual ~ConstraintEpipolar();

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const{return JAC_ANALYTIC;}

        /** \brief Returns a vector of scalar pointers to the first element of all state blocks involved in the constraint
         **/
        virtual const std::vector<Scalar*> getStateBlockPtrVector(){return std::vector<Scalar*>(0);}

        /** \brief Returns a vector of pointers to the states in which this constraint depends
         **/
        virtual const std::vector<StateBlockPtr> getStatePtrVector() const{return std::vector<StateBlockPtr>(0);}

        /** \brief Returns the constraint residual size
         **/
        virtual unsigned int getSize() const{return 0;}

    public:
        static wolf::ConstraintBasePtr create(FeatureBasePtr _feature_ptr, //
                NodeBasePtr _correspondant_ptr);

};

inline ConstraintEpipolar::ConstraintEpipolar(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr, bool _apply_loss_function, ConstraintStatus _status) :
        ConstraintBase(CTR_EPIPOLAR, _feature_other_ptr, _apply_loss_function, _status)
{
    setType("EPIPOLAR");
}

inline ConstraintEpipolar::~ConstraintEpipolar(){}

inline wolf::ConstraintBasePtr ConstraintEpipolar::create(FeatureBasePtr _feature_ptr, //
        NodeBasePtr _correspondant_ptr)
{
    return std::make_shared<ConstraintEpipolar>(_feature_ptr, std::static_pointer_cast<FeatureBase>(_correspondant_ptr));
}

} // namespace wolf

#endif // CONSTRAINT_EPIPOLAR_H
