
#ifndef CONSTRAINT_FIX_H_
#define CONSTRAINT_FIX_H_

//Wolf includes
#include "constraint_sparse.h"
#include "frame_base.h"
#include "rotations.h"


namespace wolf {
    
WOLF_PTR_TYPEDEFS(ConstraintFix);

//class
class ConstraintFix: public ConstraintSparse<3,2,1>
{
    public:
        ConstraintFix(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<3, 2, 1>(CTR_FIX, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(),
                                          _ftr_ptr->getFramePtr()->getOPtr())
        {
            setType("FIX");
            //std::cout << "creating ConstraintFix " << std::endl;
        }

        virtual ~ConstraintFix()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _p, const T* const _o, T* _residuals) const;

        /** \brief Returns the jacobians computation method
         *
         * Returns the jacobians computation method
         *
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }

};

template<typename T>
inline bool ConstraintFix::operator ()(const T* const _p, const T* const _o, T* _residuals) const
{
    // measurement
    Eigen::Matrix<T,3,1> meas =  getMeasurement().cast<T>();

    // error
    Eigen::Matrix<T,3,1> er;
    er(0) = meas(0) - _p[0];
    er(1) = meas(1) - _p[1];
    er(2) = meas(2) - _o[0];
    while (er[2] > T(M_PI))
        er(2) = er(2) - T(2*M_PI);
    while (er(2) <= T(-M_PI))
        er(2) = er(2) + T(2*M_PI);

    // residual
    Eigen::Map<Eigen::Matrix<T,3,1>> res(_residuals);
    res = getFeaturePtr()->getMeasurementSquareRootInformation().cast<T>() * er;

    return true;
}

} // namespace wolf

#endif
