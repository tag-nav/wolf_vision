#ifndef _CONSTRAINT_AUTODIFF_APRILTAG_H_
#define _CONSTRAINT_AUTODIFF_APRILTAG_H_

//Wolf includes
#include "wolf.h"
#include "rotations.h"
#include "constraint_autodiff.h"
#include "sensor_base.h"
#include "landmark_apriltag.h"
#include "features/feature_apriltag.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(ConstraintAutodiffApriltag);

class ConstraintAutodiffApriltag : public ConstraintAutodiff<ConstraintAutodiffApriltag, 6, 3, 4, 3, 4, 3, 4>
{
    public:

        /** \brief Class constructor
         */
        ConstraintAutodiffApriltag(
                const SensorBasePtr& _sensor_ptr,
                const FrameBasePtr& _frame_ptr,
                const LandmarkApriltagPtr& _landmark_other_ptr,
                const FeatureApriltagPtr& _feature_ptr,
                bool _apply_loss_function,
                ConstraintStatus _status);

        /** \brief Class Destructor
         */
        virtual ~ConstraintAutodiffApriltag();
 
        /** brief : compute the residual from the state blocks being iterated by the solver.
         **/
        template<typename T>
        bool operator ()( const T* const _p_camera, const T* const _o_camera, const T* const _p_keyframe, const T* const _o_keyframe, const T* const _p_landmark, const T* const _o_landmark, T* _residuals) const;

};

} // namespace wolf
 
// Include here all headers for this class
//#include <YOUR_HEADERS.h>

namespace wolf
{

ConstraintAutodiffApriltag::ConstraintAutodiffApriltag(
        const SensorBasePtr& _sensor_ptr,
        const FrameBasePtr& _frame_ptr,
        const LandmarkApriltagPtr& _landmark_other_ptr,
        const FeatureApriltagPtr& _feature_ptr,
        bool _apply_loss_function,
        ConstraintStatus _status) :
            ConstraintAutodiff("APRILTAG",
                               nullptr,
                               nullptr,
                               nullptr,
                               _landmark_other_ptr,
                               nullptr,
                               false,
                               CTR_ACTIVE,
                               _sensor_ptr->getPPtr(), _sensor_ptr->getOPtr(),
                               _frame_ptr->getPPtr(), _frame_ptr->getOPtr(),
                               _landmark_other_ptr->getPPtr(), _landmark_other_ptr->getOPtr()
                               )
{


}

/** \brief Class Destructor
 */
ConstraintAutodiffApriltag::~ConstraintAutodiffApriltag()
{
    //
}

template<typename T> bool ConstraintAutodiffApriltag::operator ()( const T* const _p_camera, const T* const _o_camera, const T* const _p_keyframe, const T* const _o_keyframe, const T* const _p_landmark, const T* const _o_landmark, T* _residuals) const
{
    //states
    Eigen::Translation<T,3> p_camera(_p_camera[0], _p_camera[1], _p_camera[2]),
                            p_keyframe(_p_keyframe[0], _p_keyframe[1], _p_keyframe[2]),
                            p_landmark(_p_landmark[0], _p_landmark[1], _p_landmark[2]);
    Eigen::Quaternion<T> q_camera(_o_camera), q_keyframe(_o_keyframe), q_landmark(_o_landmark);

    //Measurements
    Eigen::Translation<Scalar, 3>  p_measured(getMeasurement()(0), getMeasurement()(1), getMeasurement()(2));
    Eigen::Quaternions     q_measured( getMeasurement().data() + 3 );

    // Create transformation matrices to compose
    // camera wrt robot
    Eigen::Transform<T, 3, Eigen::Affine> r_M_c = p_camera * q_camera;
    // robot wrt world
    Eigen::Transform<T, 3, Eigen::Affine> w_M_r = p_keyframe * q_keyframe;
    // landmark wrt world
    Eigen::Transform<T, 3, Eigen::Affine> w_M_l = p_landmark * q_landmark;
    // landmark wrt camera, measure
    Eigen::Transform<T, 3, Eigen::Affine> c_M_l_meas = p_measured.cast<T>() * q_measured.cast<T>();

    // landmark wrt camera, estimated
    Eigen::Transform<T, 3, Eigen::Affine> c_M_l_est = (w_M_r * r_M_c).inverse() * w_M_l;

    // expectation error, in camera frame
    Eigen::Transform<T, 3, Eigen::Affine> c_M_err = c_M_l_meas * c_M_l_est.inverse(); // left-minus gives error is the reference camera

    // error
    Eigen::Matrix<T, 6, 1> er;
    er.block(0,0,3,1) = c_M_err.translation();
    Eigen::Matrix<T, 3, 3> R_err(c_M_err.linear());
    er.block(3,0,3,1) = wolf::log_R(R_err);  // BUG

    // residual
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(_residuals);
    res = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    return true;
}

} // namespace wolf

#endif /* _CONSTRAINT_AUTODIFF_APRILTAG_H_ */
