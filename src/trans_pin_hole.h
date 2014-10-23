/**
 * \file trans_pin_hole.h
 *
 *  Created on: 31/08/2014
 *     \author: jsola
 */

#ifndef TRANS_PIN_HOLE_H_
#define TRANS_PIN_HOLE_H_

#include "trans_sensor.h"

/**
 * \brief TransSensor involving two pin-hole sensors.
 */
class TransPinHole : public TransSensor
{

    private:
        // We access derived sensors through these pointers defined at construction time
        // via a dynamic_cast to the correct type.
        PinHolePtr pin_hole_own_ptr_; ///< pointer to this pin hole sensor
        PinHolePtr pin_hole_other_ptr_; ///< pointer to other pin hole sensor

        Eigen::Matrix3s skew_, ///< The skew symmetric matrix of the relative translation.
        essential_, ///< the essential matrix
        fundamental_; ///< the fundamental matrix

    public:

        TransPinHole(const CapturePtr& _capt_own_ptr, const CapturePtr& _capt_other_ptr);

        virtual ~TransPinHole();

        /**
         * \brief Compute Essential and Fundamental matrices.
         *
         * These matrices remain constant as long as the relative pose between captures does not change.
         */
        virtual void precomputeConstants();

        /**
         * \brief Compute the expected epipolar line in the own camera, from the pixel measured in the other camera.
         * @param _pixel_other the pixel measured in the other camera
         * @param _epi_line the epipolar line in the own camera. This is the return value.
         */
        virtual void computeExpectation(const Eigen::VectorXs & _pixel_other, Eigen::VectorXs & _epi_line) const;

};

//////////////////////////
// IMPLEMENTATION
//////////////////////////

#include "wolf_tools.h"

inline TransPinHole::TransPinHole(const CapturePtr& _capt_own_ptr, const CapturePtr& _capt_other_ptr) :
        TransSensor(_capt_own_ptr, _capt_other_ptr), //
        pin_hole_own_ptr_(dynamic_cast<PinHolePtr>(_capt_own_ptr->sensorPtr() ) ), //
        pin_hole_other_ptr_(dynamic_cast<PinHolePtr>(_capt_other_ptr->sensorPtr() ) )
{
    // NOTE: We allow the slower but safer dynamic_cast because this constructor will be called just once for each TransPinHole instance in the Wolf tree.
}

inline TransPinHole::~TransPinHole()
{
    //
}

inline void TransPinHole::precomputeConstants()
{
    precomputePoseRelative();
    Wolf::skew( poseRelative().p(), skew_ );
    // helper: F = (K_own^-1)' * R' * [T]_x * K_other^-1;
    fundamental_ = // split lines for debugging convenience
            pin_hole_own_ptr_->inverseIntrinsicMatrix().transpose() //
            * poseRelative().q().matrix() //
            * skew_ //
            * pin_hole_other_ptr_->inverseIntrinsicMatrix();
}

inline void TransPinHole::computeExpectation(const Eigen::VectorXs & _pixel_other, Eigen::VectorXs & _epi_line) const
{
    // This line is equivalent and faster than doing fundamental_ * _pixel_other.homogeneous(), i.e., line = F * hmg_point.
    _epi_line = fundamental_.leftCols(2) * _pixel_other + fundamental_.rightCols(1);
}

#endif /* TRANS_PIN_HOLE_H_ */
