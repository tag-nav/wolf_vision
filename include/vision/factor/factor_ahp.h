//--------LICENSE_START--------
//
// Copyright (C) 2020,2021 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
#ifndef FACTOR_AHP_H
#define FACTOR_AHP_H

//Wolf includes
#include "vision/landmark/landmark_ahp.h"
#include "vision/sensor/sensor_camera.h"
#include "vision/math/pinhole_tools.h"

#include "core/factor/factor_autodiff.h"

#include <iomanip> //setprecision

namespace wolf {

WOLF_PTR_TYPEDEFS(FactorAhp);
    
//class    
class FactorAhp : public FactorAutodiff<FactorAhp, 2, 3, 4, 3, 4, 4>
{
    protected:
        Eigen::Vector3d anchor_sensor_extrinsics_p_;
        Eigen::Vector4d anchor_sensor_extrinsics_o_;
        Eigen::Vector4d intrinsic_;
        Eigen::VectorXd distortion_;

    public:

        FactorAhp(const FeatureBasePtr&   _ftr_ptr,
                  const LandmarkAhpPtr&   _landmark_ptr,
                  const ProcessorBasePtr& _processor_ptr,
                  bool              _apply_loss_function,
                  FactorStatus  _status = FAC_ACTIVE);

        ~FactorAhp() override = default;

        template<typename T>
        void expectation(const T* const _current_frame_p,
                         const T* const _current_frame_o,
                         const T* const _anchor_frame_p,
                         const T* const _anchor_frame_o,
                         const T* const _lmk_hmg,
                         T* _expectation) const;

        Eigen::VectorXd expectation() const;

        template<typename T>
        bool operator ()(const T* const _current_frame_p,
                         const T* const _current_frame_o,
                         const T* const _anchor_frame_p,
                         const T* const _anchor_frame_o,
                         const T* const _lmk_hmg,
                         T* _residuals) const;
};

inline FactorAhp::FactorAhp(const FeatureBasePtr&   _ftr_ptr,
                            const LandmarkAhpPtr&   _landmark_ptr,
                            const ProcessorBasePtr& _processor_ptr,
                            bool             _apply_loss_function,
                            FactorStatus _status) :
        FactorAutodiff<FactorAhp, 2, 3, 4, 3, 4, 4>("AHP",
                                                    TOP_LMK,
                                                    _ftr_ptr,
                                                    _landmark_ptr->getAnchorFrame(),
                                                    nullptr,
                                                    nullptr,
                                                    _landmark_ptr,
                                                    _processor_ptr,
                                                    _apply_loss_function,
                                                    _status,
                                                    _ftr_ptr->getCapture()->getFrame()->getP(),
                                                    _ftr_ptr->getCapture()->getFrame()->getO(),
                                                    _landmark_ptr->getAnchorFrame()->getP(),
                                                    _landmark_ptr->getAnchorFrame()->getO(),
                                                    _landmark_ptr->getP()),
        anchor_sensor_extrinsics_p_(_ftr_ptr->getCapture()->getSensorP()->getState()),
        anchor_sensor_extrinsics_o_(_ftr_ptr->getCapture()->getSensorO()->getState()),
        intrinsic_(_ftr_ptr->getCapture()->getSensor()->getIntrinsic()->getState())
{
    // obtain some intrinsics from provided sensor
    distortion_ = (std::static_pointer_cast<SensorCamera>(_ftr_ptr->getCapture()->getSensor()))->getDistortionVector();
}

inline Eigen::VectorXd FactorAhp::expectation() const
{
    FrameBasePtr frm_current = getFeature()->getCapture()->getFrame();
    FrameBasePtr frm_anchor  = getFrameOther();
    LandmarkBasePtr lmk      = getLandmarkOther();

    const Eigen::MatrixXd frame_current_pos = frm_current->getP()->getState();
    const Eigen::MatrixXd frame_current_ori = frm_current->getO()->getState();
    const Eigen::MatrixXd frame_anchor_pos  = frm_anchor ->getP()->getState();
    const Eigen::MatrixXd frame_anchor_ori  = frm_anchor ->getO()->getState();
    const Eigen::MatrixXd lmk_pos_hmg       = lmk        ->getP()->getState();

    Eigen::Vector2d exp;
    expectation(frame_current_pos.data(), frame_current_ori.data(), frame_anchor_pos.data(),
                frame_anchor_ori.data(), lmk_pos_hmg.data(), exp.data());

    return exp;
}

template<typename T>
inline void FactorAhp::expectation(const T* const _current_frame_p,
                                   const T* const _current_frame_o,
                                   const T* const _anchor_frame_p,
                                   const T* const _anchor_frame_o,
                                   const T* const _lmk_hmg,
                                   T* _expectation) const
{
    using namespace Eigen;

    // All involved transforms typedef
    typedef Eigen::Transform<T, 3, Eigen::Isometry> TransformType;

    // world to anchor robot transform
    Map<const Matrix<T, 3, 1> > p_w_r0(_anchor_frame_p);
    Translation<T, 3>           t_w_r0(p_w_r0);
    Map<const Quaternion<T> >   q_w_r0(_anchor_frame_o);
    TransformType               T_W_R0 = t_w_r0 * q_w_r0;

    // world to current robot transform
    Map<const Matrix<T, 3, 1> > p_w_r1(_current_frame_p);
    Translation<T, 3>           t_w_r1(p_w_r1);
    Map<const Quaternion<T> >   q_w_r1(_current_frame_o);
    TransformType               T_W_R1 = t_w_r1 * q_w_r1;

    // anchor robot to anchor camera transform
    Translation<T, 3>   t_r0_c0(anchor_sensor_extrinsics_p_.cast<T>());
    Quaternion<T>       q_r0_c0(anchor_sensor_extrinsics_o_.cast<T>());
    TransformType       T_R0_C0 = t_r0_c0 * q_r0_c0;

    // current robot to current camera transform
    CaptureBasePtr      current_capture = this->getFeature()->getCapture();
    Translation<T, 3>   t_r1_c1  (current_capture->getSensorP()->getState().cast<T>());
    Quaterniond         q_r1_c1_s(Eigen::Vector4d(current_capture->getSensorO()->getState()));
    Quaternion<T>       q_r1_c1 = q_r1_c1_s.cast<T>();
    TransformType       T_R1_C1 = t_r1_c1 * q_r1_c1;

    // hmg point in current camera frame C1
    Eigen::Map<const Eigen::Matrix<T, 4, 1> > landmark_hmg_c0(_lmk_hmg);
    Eigen::Matrix<T, 4, 1> landmark_hmg_c1 = T_R1_C1.inverse(Eigen::Isometry)
                                           * T_W_R1. inverse(Eigen::Isometry)
                                           * T_W_R0
                                           * T_R0_C0
                                           * landmark_hmg_c0;

    // lmk direction vector
    Eigen::Matrix<T, 3, 1> v_dir = landmark_hmg_c1.head(3);

    // camera parameters
    Matrix<T, 4, 1> intrinsic = intrinsic_.cast<T>();
    Eigen::Matrix<T, Eigen::Dynamic, 1> distortion = distortion_.cast<T>();

    // project point and exit
    Eigen::Map<Eigen::Matrix<T, 2, 1> > expectation(_expectation);
    expectation = pinhole::projectPoint(intrinsic, distortion, v_dir);
}

template<typename T>
inline bool FactorAhp::operator ()(const T* const _current_frame_p,
                                   const T* const _current_frame_o,
                                   const T* const _anchor_frame_p,
                                   const T* const _anchor_frame_o,
                                   const T* const _lmk_hmg,
                                   T* _residuals) const
{
    // expected
    Eigen::Matrix<T, 2, 1> expected;
    expectation(_current_frame_p, _current_frame_o, _anchor_frame_p, _anchor_frame_o, _lmk_hmg, expected.data());

    // measured
    Eigen::Matrix<T, 2, 1> measured = getMeasurement().cast<T>();

    // residual
    Eigen::Map<Eigen::Matrix<T, 2, 1> > residuals(_residuals);
    residuals = getMeasurementSquareRootInformationUpper().cast<T>() * (expected - measured);
    return true;
}

} // namespace wolf

#endif // FACTOR_AHP_H
