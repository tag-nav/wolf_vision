#ifndef _CONSTRAINT_AUTODIFF_APRILTAG_H_
#define _CONSTRAINT_AUTODIFF_APRILTAG_H_

//Wolf includes
#include "wolf.h"
#include "constraint_autodiff.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(ConstraintAutodiffApriltag);

class ConstraintAutodiffApriltag : public ConstraintAutodiff<ConstraintAutodiffApriltag, 6, 3, 4, 3, 4, 3, 4>
{
    public:

        /** \brief Class constructor
         */
        // TODO Modify this default API to suit your class needs
        ConstraintAutodiffApriltag(const std::string& _tp, const FrameBasePtr& _frame_other_ptr, const CaptureBasePtr& _capture_other_ptr, const FeatureBasePtr& _feature_other_ptr, const LandmarkBasePtr& _landmark_other_ptr, const ProcessorBasePtr& _processor_ptr, bool _apply_loss_function, ConstraintStatus _status, StateBlockPtr _state0Ptr, StateBlockPtr _state1Ptr, StateBlockPtr _state2Ptr, StateBlockPtr _state3Ptr, StateBlockPtr _state4Ptr, StateBlockPtr _state5Ptr, StateBlockPtr _state6Ptr, StateBlockPtr _state7Ptr, StateBlockPtr _state8Ptr, StateBlockPtr _state9Ptr);

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

template<typename T> bool ConstraintAutodiffApriltag::operator ()( const T* const _p_camera, const T* const _o_camera, const T* const _p_keyframe, const T* const _o_keyframe, const T* const _p_landmark, const T* const _o_landmark, T* _residuals) const
{
    //states
    Eigen::Translation<T,3,1> p_camera(_p_camera), p_keyframe(_p_keyframe), p_landmark(_p_landmark);
    Eigen::Quaternion<T> q_camera(_o_camera), q_keyframe(_o_keyframe), q_landmark(_o_landmark);

    //Measurements
    Eigen::Translation<T,3,1>  p_measured(getMeasurement().data() + 0);
    Eigen::Quaternion<T>       q_measured(getMeasurement().data() + 3);

    // 1. create transformation matrix to compose
    Transform<T, 3, Affine> r_M_c = p_camera * q_camera;
    Transform<T, 3, Affine> w_M_r = p_keyframe * q_keyframe;
    Transform<T, 3, Affine> w_M_l = p_landmark * q_landmark;
    Transform<T, 3, Affine> l_M_c_meas = p_measured * q_measured;

    Transform<T, 3, Affine> w_M_c = w_M_r * r_M_c;
    Transform<T, 3, Affine> w_M_c_meas = w_M_l * l_M_c_meas;
    Transform<T, 3, Affine> c_M_c_meas = w_M_c.inverse() *  w_M_c_meas;

    //error
    Eigen::Matrix<T, 6, 1> er;
    er.head<3>() = c_M_c_meas.translation();
    er.tail(3)() = minus_right(q_measured, q_estimated); //TODO: q_estimated

    // residual
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(_residuals);
    res               = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * er;

    return true;
}

} // namespace wolf

#endif /* _CONSTRAINT_AUTODIFF_APRILTAG_H_ */
