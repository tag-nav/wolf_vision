#ifndef CONSTRAINT_ODOM_2D_THETA_H_
#define CONSTRAINT_ODOM_2D_THETA_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"

class ConstraintOdom2D : public ConstraintSparse<3, 2, 1, 2, 1>
{
    public:
        static const unsigned int N_BLOCKS = 4;

        ConstraintOdom2D(FeatureBase* _ftr_ptr, FrameBase* _frame_ptr, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<3, 2, 1, 2, 1>(_ftr_ptr, CTR_ODOM_2D, _frame_ptr, _status, _frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _ftr_ptr->getFramePtr()->getPPtr(), _ftr_ptr->getFramePtr()->getOPtr())
        {
            //
        }

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintOdom2D()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _p1, const T* const _o1, const T* const _p2, const T* const _o2,
                         T* _residuals) const;

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
inline bool ConstraintOdom2D::operator ()(const T* const _p1, const T* const _o1, const T* const _p2,
                                          const T* const _o2, T* _residuals) const
{
    //std::cout << "computing constraint odom ..." << std::endl;
    //            std::cout << "_p1: ";
    //            for (int i=0; i < 2; i++)
    //                std::cout << "\n\t" << _p1[i];
    //            std::cout << std::endl;
    //            std::cout << "_o1: ";
    //            for (int i=0; i < 1; i++)
    //                std::cout << "\n\t" << _o1[i];
    //            std::cout << std::endl;
    //            std::cout << "_p2: ";
    //            for (int i=0; i < 2; i++)
    //                std::cout << "\n\t" << _p2[i];
    //            std::cout << std::endl;
    //            std::cout << "_o2: ";
    //            for (int i=0; i < 1; i++)
    //                std::cout << "\n\t" << _o2[i];
    //            std::cout << std::endl;
    //            std::cout << "getMeasurement(): ";
    //            for (int i=0; i < 3; i++)
    //                std::cout << "\n\t" << getMeasurement()(i);
    //            std::cout << std::endl;
    //            std::cout << "getMeasurementCovariance(): ";
    //            for (int i=0; i < 3; i++)
    //                std::cout << "\n\t" << getMeasurementCovariance()(i,i);
    //            std::cout << std::endl;
    // Expected measurement
    // rotar menys l'angle de primer (-_o1)
    T expected_longitudinal = cos(_o1[0]) * (_p2[0] - _p1[0]) + sin(_o1[0]) * (_p2[1] - _p1[1]); // cos(-o1)(x2-x1) - sin(-o1)(y2-y1)
    T expected_lateral = -sin(_o1[0]) * (_p2[0] - _p1[0]) + cos(_o1[0]) * (_p2[1] - _p1[1]); // sin(-o1)(x2-x1) + cos(-o1)(y2-y1)
    T expected_rotation = _o2[0] - _o1[0];
    // Residuals
    _residuals[0] = (expected_longitudinal - T(getMeasurement()(0)))
            / T(sqrt(std::max(getMeasurementCovariance()(0, 0), 1e-6)));
    _residuals[1] = (expected_lateral - T(getMeasurement()(1)))
            / T(sqrt(std::max(getMeasurementCovariance()(1, 1), 1e-6)));
    _residuals[2] = expected_rotation - T(getMeasurement()(2));
    while (_residuals[2] > T(M_PI))
        _residuals[2] = _residuals[2] - T(2 * M_PI);
    while (_residuals[2] <= T(-M_PI))
        _residuals[2] = _residuals[2] + T(2 * M_PI);
    _residuals[2] = _residuals[2] / T(sqrt(std::max(getMeasurementCovariance()(2, 2), 1e-6)));
    //std::cout << "constraint odom computed!" << std::endl;
    return true;
}

#endif
