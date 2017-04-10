
#ifndef CONSTRAINT_FIX_3D_H_
#define CONSTRAINT_FIX_3D_H_

//Wolf includes
#include "constraint_sparse.h"
#include "frame_base.h"
#include "rotations.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(ConstraintFix3D);

//class
class ConstraintFix3D: public ConstraintSparse<6,3,4>
{
    public:

        ConstraintFix3D(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<6,3,4>(CTR_FIX_3D, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(),
                                          _ftr_ptr->getFramePtr()->getOPtr())
        {
            setType("FIX3D");
        }
        virtual ~ConstraintFix3D()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _p, const T* const _o, T* _residuals) const;

        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }

};

template<typename T>
inline bool ConstraintFix3D::operator ()(const T* const _p, const T* const _o, T* _residuals) const
{

    // states
    Eigen::Matrix<T, 3, 1>  p(_p);
    Eigen::Quaternion<T>    q(_o);

    // measurements
    Eigen::Vector3s     p_measured(getMeasurement().data() + 0);
    Eigen::Quaternions  q_measured(getMeasurement().data() + 3);

    // error
    Eigen::Matrix<T, 6, 1> er;
    er.head(3)        = p_measured.cast<T>() - p;
    er.tail(3)        = q2v(q.conjugate() * q_measured.cast<T>());

    // residual
    Eigen::Map<Eigen::Matrix<T, 6, 1>> res(_residuals);
    res               = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    return true;
}

} // namespace wolf

#endif
