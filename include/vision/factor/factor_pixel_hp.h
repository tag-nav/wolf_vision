// WOLF - Copyright (C) 2020,2021,2022,2023
// Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu) and
// Joan Vallvé Navarro (jvallve@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF: http://www.iri.upc.edu/wolf
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

#pragma once

//Wolf includes
#include "vision/common/vision.h"
#include "vision/landmark/landmark_hp.h"
#include "vision/sensor/sensor_camera.h"
#include "vision/math/pinhole_tools.h"

#include "core/factor/factor_autodiff.h"

#include <iomanip> //setprecision

namespace wolf {

WOLF_PTR_TYPEDEFS(FactorPixelHp);
    
//class    
class FactorPixelHp : public FactorAutodiff<FactorPixelHp, 2, 3, 4, 3, 4, 4>
{
    protected:

        Eigen::Vector4d intrinsic_;
        Eigen::VectorXd distortion_;

    public:

        FactorPixelHp(const FeatureBasePtr&   _ftr_ptr,
                      const LandmarkHpPtr&    _landmark_ptr,
                      const ProcessorBasePtr& _processor_ptr,
                      bool                    _apply_loss_function,
                      FactorStatus            _status = FAC_ACTIVE);

        ~FactorPixelHp() override = default;

        template<typename T>
        void expectation(const T* const _frame_p,
                         const T* const _frame_o,
                         const T* const _sensor_p,
                         const T* const _sensor_o,
                         const T* const _lmk_hmg,
                         T* _expectation) const;

        Eigen::VectorXd expectation() const;

        template<typename T>
        bool operator ()(const T* const _frame_p,
                         const T* const _frame_o,
                         const T* const _sensor_p,
                         const T* const _sensor_o,
                         const T* const _lmk_hmg,
                         T* _residuals) const;
};

inline FactorPixelHp::FactorPixelHp(const FeatureBasePtr&   _ftr_ptr,
                                    const LandmarkHpPtr&   _landmark_ptr,
                                    const ProcessorBasePtr& _processor_ptr,
                                    bool             _apply_loss_function,
                                    FactorStatus _status) :
        FactorAutodiff<FactorPixelHp, 2, 3, 4, 3, 4, 4>("PIXELHP",
                                                        TOP_LMK,
                                                        _ftr_ptr,
                                                        nullptr,
                                                        nullptr,
                                                        nullptr,
                                                        _landmark_ptr,
                                                        _processor_ptr,
                                                        _apply_loss_function,
                                                        _status,
                                                        _ftr_ptr->getCapture()->getFrame()->getP(),
                                                        _ftr_ptr->getCapture()->getFrame()->getO(),
                                                        _ftr_ptr->getCapture()->getSensorP(),
                                                        _ftr_ptr->getCapture()->getSensorO(),
                                                        _landmark_ptr->getP()),
        intrinsic_(_ftr_ptr->getCapture()->getSensor()->getIntrinsic()->getState())
{
    distortion_ = (std::static_pointer_cast<SensorCamera>(_ftr_ptr->getCapture()->getSensor()))->getDistortionVector();
}

inline Eigen::VectorXd FactorPixelHp::expectation() const
{
    auto frm  = getFeature()->getCapture()->getFrame();
    auto sen  = getFeature()->getCapture()->getSensor();
    auto lmk  = getLandmarkOther();

    const Eigen::MatrixXd frame_pos     = frm->getP()->getState();
    const Eigen::MatrixXd frame_ori     = frm->getO()->getState();
    const Eigen::MatrixXd sensor_pos    = sen->getP()->getState();
    const Eigen::MatrixXd sensor_ori    = sen->getO()->getState();
    const Eigen::MatrixXd lmk_pos_hmg   = lmk->getP()->getState();

    Eigen::Vector2d exp;
    expectation(frame_pos.data(),
                frame_ori.data(),
                sensor_pos.data(),
                sensor_ori.data(),
    			lmk_pos_hmg.data(),
    			exp.data());

    return exp;
}

template<typename T>
inline void FactorPixelHp::expectation(const T* const _frame_p,
                                       const T* const _frame_o,
                                       const T* const _sensor_p,
                                       const T* const _sensor_o,
                                       const T* const _lmk_hmg,
                                       T* _expectation) const
{
    using namespace Eigen;

    // frames w: world; r: robot; c: camera
    Map<const Matrix<T, 3, 1> > p_wr(_frame_p);
    Map<const Quaternion<T> >   q_wr(_frame_o);
    Map<const Matrix<T, 3, 1> > p_rc(_sensor_p);
    Map<const Quaternion<T> >   q_rc(_sensor_o);

    // camera pose in world frame: transforms from camera to world
    Matrix<T, 3, 1> p_wc = p_wr + q_wr * p_rc;
    Quaternion<T>   q_wc = q_wr*q_rc;

    // invert transform: from world to camera
    // | R T |.inv = | R.tr -R.tr*T | == | q.conj -q.conj*T |
    // | 0 1 |       |  0      1    |    |   0        1     |
    Quaternion<T>   q_cw = q_wc.conjugate();
    Matrix<T, 3, 1> p_cw = - (q_cw * p_wc);

    // landmark hmg
    Matrix<T, 4, 1> lh_w(_lmk_hmg);

    // landmark dir vector in C frame
    /* note: transforming hmg point to get direction vector v':
     * | q T | * | v | = | q*v + T+w | --> v' = q*v + T+w
     * | 0 1 |   | w |   |     w     |
     */
    Matrix<T, 3, 1> v_dir = q_cw * lh_w.template head<3>() + p_cw * lh_w(3);

    // project point and exit
    Eigen::Map<Eigen::Matrix<T, 2, 1> > expectation(_expectation);
    expectation = pinhole::projectPoint(intrinsic_, distortion_, v_dir);

}

template<typename T>
inline bool FactorPixelHp::operator ()(const T* const _frame_p,
                                       const T* const _frame_o,
                                       const T* const _sensor_p,
                                       const T* const _sensor_o,
                                       const T* const _lmk_hmg,
                                       T* _residuals) const
{
    // expected
    Eigen::Matrix<T, 2, 1> expected;
    expectation(_frame_p, _frame_o, _sensor_p, _sensor_o, _lmk_hmg, expected.data());

    // residual
    Eigen::Map<Eigen::Matrix<T, 2, 1> > residuals(_residuals);
    residuals = getMeasurementSquareRootInformationUpper() * (expected - getMeasurement());

    return true;
}

} // namespace wolf