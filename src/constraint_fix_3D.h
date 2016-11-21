
#ifndef CONSTRAINT_FIX_3D_H_
#define CONSTRAINT_FIX_3D_H_

//Wolf includes
#include "constraint_sparse.h"
#include "frame_base.h"
#include "rotations.h"


namespace wolf {

class ConstraintFix3D: public ConstraintSparse<6,3,4>
{
    public:

        ConstraintFix3D(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<6,3,4>(CTR_FIX_3D, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(),
                                          _ftr_ptr->getFramePtr()->getOPtr())
        {
            setType("FIX3D");
            //std::cout << "creating ConstraintFix " << std::endl;
        }
        virtual ~ConstraintFix3D()
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
inline bool ConstraintFix3D::operator ()(const T* const _p, const T* const _o, T* _residuals) const
{
    //std::cout << "computing constraint odom ..." << std::endl;

    // Position
    _residuals[0] = (T(getMeasurement()(0)) - _p[0]) / T(sqrt(getMeasurementCovariance()(0, 0)));
    _residuals[1] = (T(getMeasurement()(1)) - _p[1]) / T(sqrt(getMeasurementCovariance()(1, 1)));
    _residuals[2] = (T(getMeasurement()(2)) - _p[2]) / T(sqrt(getMeasurementCovariance()(2, 2)));

    // Orientation
//    Eigen::Map<Eigen::Quaternion<T> > q(_o);
//    Eigen::Map<Eigen::Quaternions> expected_q(getMeasurement().data()+3);

//    Eigen::Quaternion<T> dq = q.conjugate() * expected_q.cast<T>();
//    Eigen::Matrix<T,3,1> dtheta = q2v(dq);

    Eigen::Quaternion<T> q(_o);
    Eigen::Quaternions expected_q(getMeasurement().data()+3);

    Eigen::Quaternion<T> dq = q.conjugate() * expected_q.cast<T>();
    Eigen::Matrix<T,3,1> dtheta = q2v(dq);

    _residuals[3] = dtheta(0);
    _residuals[4] = dtheta(1);
    _residuals[5] = dtheta(2);

    //            std::cout << "+++++++  fix constraint +++++++" << std::endl;
    //            std::cout << "orientation:   " << _o[0] << std::endl;
    //            std::cout << "measurement:   " << T(getMeasurement()(2)) << std::endl;
    //            std::cout << "residual:      " << _residuals[2] << std::endl;
    //            std::cout << "is > PI        " << bool(_residuals[2] > T(2*M_PI)) << std::endl;
    //            std::cout << "is >= PI       " << bool(_residuals[2] <= T(-2*M_PI)) << std::endl;

    while (_residuals[3] > T(M_PI))
        _residuals[3] = _residuals[3] - T(2 * M_PI);
    while (_residuals[3] <= T(-M_PI))
        _residuals[3] = _residuals[3] + T(2 * M_PI);

    while (_residuals[4] > T(M_PI))
        _residuals[4] = _residuals[4] - T(2 * M_PI);
    while (_residuals[4] <= T(-M_PI))
        _residuals[4] = _residuals[4] + T(2 * M_PI);

    while (_residuals[5] > T(M_PI))
        _residuals[5] = _residuals[5] - T(2 * M_PI);
    while (_residuals[5] <= T(-M_PI))
        _residuals[5] = _residuals[5] + T(2 * M_PI);

    //            std::cout << "residual:      " << _residuals[2] << std::endl << std::endl;
    _residuals[3] = _residuals[3] / T(sqrt(getMeasurementCovariance()(3, 3)));
    _residuals[4] = _residuals[4] / T(sqrt(getMeasurementCovariance()(4, 4)));
    _residuals[5] = _residuals[5] / T(sqrt(getMeasurementCovariance()(5, 5)));
    //std::cout << "constraint fix computed!" << std::endl;
    return true;
}

} // namespace wolf

#endif
