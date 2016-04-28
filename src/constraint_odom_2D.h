#ifndef CONSTRAINT_ODOM_2D_THETA_H_
#define CONSTRAINT_ODOM_2D_THETA_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"

namespace wolf {

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

    // MAPS
    Eigen::Map<Eigen::Matrix<T,3,1> > residuals_map(_residuals);
    Eigen::Map<const Eigen::Matrix<T,2,1> > p1_map(_p1);
    Eigen::Map<const Eigen::Matrix<T,2,1> > p2_map(_p2);
    Eigen::Matrix<T, 3, 1> expected_measurement;

    // Expected measurement
    expected_measurement.head(2) = Eigen::Rotation2D<T>(-_o1[0]) * (p2_map - p1_map);
    expected_measurement(2) = _o2[0] - _o1[0];

    // Error
    residuals_map = expected_measurement - getMeasurement().cast<T>();
    // pi2pi
    while (residuals_map(2) > T(Constants::PI))
        residuals_map(2) = residuals_map(2) - T(2 * Constants::PI);
    while (residuals_map(2) <= T(-Constants::PI))
        residuals_map(2) = residuals_map(2) + T(2 * Constants::PI);

    // Residuals
    residuals_map = getMeasurementSquareRootInformation().cast<T>() * residuals_map;
    // std::cout << "constraint odom computed!" << std::endl;

    return true;
}

} // namespace wolf

#endif
