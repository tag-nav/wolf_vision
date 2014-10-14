/**
 * \file pin_hole.h
 *
 *  Created on: 31/08/2014
 *     \author: jsola
 */

#ifndef PIN_HOLE_H_
#define PIN_HOLE_H_

#include "sensor_base.h"

class PinHole : public SensorBase
{
    private:
        Eigen::Matrix3s intrinsic_matrix_;
        Eigen::Matrix3s inverse_intrinsic_matrix_;

    public:
        PinHole(StatePose& _sp, Eigen::VectorXs _intrinsic, const NodeLocation _loc = MID);
        virtual ~PinHole();

        const Eigen::Matrix3s& intrinsiMatrix() const;
        const Eigen::Matrix3s& inverseIntrinsicMatrix() const;

        virtual void precomputeConstants();

    private:

        void computeIntrinsicMatrix();
        void computeInverseIntrinsicMatrix();

};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////

inline PinHole::PinHole(StatePose& _sp, Eigen::VectorXs _intrinsic, const NodeLocation _loc) :
        SensorBase(_sp, _intrinsic, _loc)
{
    precomputeConstants();
}

inline PinHole::~PinHole()
{
    // TODO
}

inline const Eigen::Matrix3s& PinHole::intrinsiMatrix() const
{
    return intrinsic_matrix_;
}

inline const Eigen::Matrix3s& PinHole::inverseIntrinsicMatrix() const
{
    return inverse_intrinsic_matrix_;
}

inline void PinHole::precomputeConstants()
{
    computeIntrinsicMatrix();
    computeInverseIntrinsicMatrix();
}

inline void PinHole::computeIntrinsicMatrix()
{
    // helper: K =
    //    [ k2,  0, k0]
    //    [  0, k3, k1]
    //    [  0,  0,  1]
    Eigen::VectorXs k = intrinsic();
    intrinsic_matrix_(0, 0) = k(2);
    intrinsic_matrix_(0, 1) = 0;
    intrinsic_matrix_(0, 2) = k(0);
    intrinsic_matrix_(1, 0) = 0;
    intrinsic_matrix_(1, 1) = k(3);
    intrinsic_matrix_(1, 2) = k(1);
    intrinsic_matrix_(2, 0) = 0;
    intrinsic_matrix_(2, 1) = 0;
    intrinsic_matrix_(2, 2) = 1;
}

inline void PinHole::computeInverseIntrinsicMatrix()
{
    // helper: inverse_intrinsic_matrix_ =
    //    [ 1/k2,    0, -k0/k2]
    //    [    0, 1/k3, -k1/k3]
    //    [    0,    0,      1]
    Eigen::VectorXs k = intrinsic();
    WolfScalar ik2 = 1 / k(2);
    WolfScalar ik3 = 1 / k(3);
    inverse_intrinsic_matrix_(0, 0) = ik2;
    inverse_intrinsic_matrix_(0, 1) = 0;
    inverse_intrinsic_matrix_(0, 2) = -k(0) * ik2;
    inverse_intrinsic_matrix_(1, 0) = 0;
    inverse_intrinsic_matrix_(1, 1) = ik3;
    inverse_intrinsic_matrix_(1, 2) = -k(1) * ik3;
    inverse_intrinsic_matrix_(2, 0) = 0;
    inverse_intrinsic_matrix_(2, 1) = 0;
    inverse_intrinsic_matrix_(2, 2) = 1;
}

#endif /* PIN_HOLE_H_ */
