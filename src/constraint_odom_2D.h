#ifndef CONSTRAINT_ODOM_2D_THETA_H_
#define CONSTRAINT_ODOM_2D_THETA_H_

//Wolf includes
#include "constraint_sparse.h"
#include "frame_base.h"

namespace wolf {
    
WOLF_PTR_TYPEDEFS(ConstraintOdom2D);
    
//class
class ConstraintOdom2D : public ConstraintSparse<3, 2, 1, 2, 1>
{
    public:
        ConstraintOdom2D(FeatureBasePtr _ftr_ptr, FrameBasePtr _frame_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<3, 2, 1, 2, 1>(CTR_ODOM_2D, _frame_ptr, nullptr, nullptr, _apply_loss_function, _status, _frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _ftr_ptr->getFramePtr()->getPPtr(), _ftr_ptr->getFramePtr()->getOPtr())
        {
            setType("ODOM 2D");
        }

        virtual ~ConstraintOdom2D()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _p1, const T* const _o1, const T* const _p2, const T* const _o2,
                         T* _residuals) const;

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }

    public:
        static wolf::ConstraintBasePtr create(FeatureBasePtr _feature_ptr, //
                                            NodeBasePtr _correspondant_ptr)
        {
            return std::make_shared<ConstraintOdom2D>(_feature_ptr, std::static_pointer_cast<FrameBase>(_correspondant_ptr) );
        }

};

template<typename T>
inline bool ConstraintOdom2D::operator ()(const T* const _p1, const T* const _o1, const T* const _p2,
                                          const T* const _o2, T* _residuals) const
{

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
    // pi2pi - cannot use wolf::pi2pi() because of template T
    while (residuals_map(2) > T(M_PI))
        residuals_map(2) = residuals_map(2) - T(2 * M_PI);
    while (residuals_map(2) <= T(-M_PI))
        residuals_map(2) = residuals_map(2) + T(2 * M_PI);

    // Residuals
    residuals_map = getMeasurementSquareRootInformation().cast<T>() * residuals_map;
    // std::cout << "constraint odom computed!" << std::endl;

    return true;
}

} // namespace wolf

#endif
