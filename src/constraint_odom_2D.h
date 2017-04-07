#ifndef CONSTRAINT_ODOM_2D_THETA_H_
#define CONSTRAINT_ODOM_2D_THETA_H_

//Wolf includes
#include "constraint_sparse.h"
#include "frame_base.h"

//#include "ceres/jet.h"

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
//            std::cout << "created ConstraintOdom2D " << std::endl;
        }

        virtual ~ConstraintOdom2D()
        {
//            std::cout << "destructed ConstraintOdom2D " << std::endl;
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
    Eigen::Map<Eigen::Matrix<T,3,1> > res(_residuals);
    Eigen::Map<const Eigen::Matrix<T,2,1> > p1(_p1);
    Eigen::Map<const Eigen::Matrix<T,2,1> > p2(_p2);
    T o1 = _o1[0];
    T o2 = _o2[0];
    Eigen::Matrix<T, 3, 1> expected_measurement;
    Eigen::Matrix<T, 3, 1> er; // error

    // Expected measurement
    expected_measurement.head(2) = Eigen::Rotation2D<T>(-o1) * (p2 - p1);
    expected_measurement(2) = o2 - o1;

    // Error
    er = expected_measurement - getMeasurement().cast<T>();
    while (er(2) > T( M_PI ))
        er(2) -=   T( 2 * M_PI );
    while (er(2) < T( -M_PI ))
        er(2) +=   T( 2 * M_PI );

    // Residuals
    res = getMeasurementSquareRootInformationTransposed().cast<T>() * er;


    ////////////////////////////////////////////////////////
    // print Jacobian. Uncomment this as you wish (remember to uncomment #include "ceres/jet.h" above):
//    using ceres::Jet;
//    Eigen::MatrixXs J(3,6);
//    J.row(0) = ((Jet<Scalar, 6>)(er(0))).v;
//    J.row(1) = ((Jet<Scalar, 6>)(er(1))).v;
//    J.row(2) = ((Jet<Scalar, 6>)(er(2))).v;
//    J.row(0) = ((Jet<Scalar, 6>)(res(0))).v;
//    J.row(1) = ((Jet<Scalar, 6>)(res(1))).v;
//    J.row(2) = ((Jet<Scalar, 6>)(res(2))).v;
//    if (sizeof(er(0)) != sizeof(double))
//    {
//        std::cout << "ConstraintOdom2D::Jacobian(c" << id() << ") = \n " << J << std::endl;
//        std::cout << "ConstraintOdom2D::Weighted Jacobian(c" << id() << ") = \n " << J << std::endl;
//        std::cout << "ConstraintOdom2D::Sqrt Info(c" << id() << ") = \n" << getMeasurementSquareRootInformationTransposed() << std::endl;
//    }
    ////////////////////////////////////////////////////////


    return true;
}

} // namespace wolf

#endif
