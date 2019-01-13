/**
 * \file constraint_autodiff_distance_3D.h
 *
 *  Created on: Apr 10, 2018
 *      \author: jsola
 */

#ifndef CONSTRAINT_AUTODIFF_DISTANCE_3D_H_
#define CONSTRAINT_AUTODIFF_DISTANCE_3D_H_

#include "constraint_autodiff.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(ConstraintAutodiffDistance3D);

class ConstraintAutodiffDistance3D : public ConstraintAutodiff<ConstraintAutodiffDistance3D, 1, 3, 3>
{
    public:
        ConstraintAutodiffDistance3D(const FeatureBasePtr&   _feat,
                                     const FrameBasePtr&     _frm_other,
                                     const ProcessorBasePtr& _processor_ptr,
                                     bool                    _apply_loss_function = false,
                                     ConstraintStatus        _status = CTR_ACTIVE) :
                                         ConstraintAutodiff("DISTANCE 3D",
                                                            _frm_other,
                                                            nullptr,
                                                            nullptr,
                                                            nullptr,
                                                            _processor_ptr,
                                                            _apply_loss_function,
                                                            _status,
                                                            _feat->getFramePtr()->getPPtr(),
                                                            _frm_other->getPPtr())
        {
            setFeaturePtr(_feat);
        }

        virtual ~ConstraintAutodiffDistance3D() { /* nothing */ }

        template<typename T>
        bool operator () (const T* const _pos1,
                                  const T* const _pos2,
                                  T* _residual) const
        {
            using namespace Eigen;

            Map<const Matrix<T,3,1>> pos1(_pos1);
            Map<const Matrix<T,3,1>> pos2(_pos2);
            Map<Matrix<T,1,1>> res(_residual);

            Matrix<T,1,1> dist_exp ( sqrt( ( pos2 - pos1 ).squaredNorm() ) );
            Matrix<T,1,1> dist_meas (getMeasurement().cast<T>() );
            Matrix<T,1,1> sqrt_info_upper = getMeasurementSquareRootInformationUpper().cast<T>();

            res  = sqrt_info_upper * (dist_meas - dist_exp);

            return true;
        }
};

} /* namespace wolf */

#endif /* CONSTRAINT_AUTODIFF_DISTANCE_3D_H_ */
