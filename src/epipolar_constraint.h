/**
 * \file epipolar_constraint.h
 *
 *  Created on: 12/08/2014
 *     \author: jsola
 */

#ifndef EPIPOLAR_CONSTRAINT_H_
#define EPIPOLAR_CONSTRAINT_H_

#include "correspondence_relative.h"

/**
 * \brief Epipolar constraint between two corresponding image point features.
 *
 * Epipolar constraint between two corresponding image point features.
 *
 * This type of constraint takes two frames, two pin-hole cameras, and two point feature measurements, and
 * computes the error of the epipolar constraint.
 *
 * Notice that there are some possibilities:
 * - Both frames are the same: it means we have two cameras on the robot.
 * - Both cameras are the same: it means we have moved and are in another frame.
 * - Frames and cameras are different: it means we observe from one camera a feature that was seen in the past by another camera.
 *
 * The epipolar error is 1D, defined as the distance from the current measurement (2D) to the epipolar line (3D)
 * defined by the old measure, the relative frame transformation, and the two sets of intrinsic parameters.
 */
class EpipolarConstraint : public CorrespondenceRelative
{
    public:

        static const unsigned int DIM_ERROR_ = 1; ///< Dimension of error
        static const unsigned int DIM_EXPECTATION_ = 3; ///< Dimension of expectation

    public:
        EpipolarConstraint(const FeatureShPtr& _ft_ptr, const FeatureShPtr& _ft_other_ptr);

        /**
         * \brief Compute the expected epipolar line
         *
         * Compute the expected epipolar line.
         *
         * The epipolar line is a homogeneous line representation, the result of projecting the visual line
         * of the pixel seen from the "other" camera into this "own" camera.
         *
         * Here is the algorithm:
         * - Consider "other" the ref camera pose, and "own" the new camera pose.
         * - Given \f$ K_{own}, K_{other} \f$ the intrinsic matrices of both cameras
         * - Given \f$ pix_{other} \f$ the detected pixel in the "other" camera,
         * - Compute R, T the relative pose from "other" to "own" cameras,
         * - Compute the Essential matrix, \f[ E = R^\top * [T]_x~. \f] Mind that the definitions in some
         *   literature (e.g. [Wikipedia](http://en.wikipedia.org/wiki/Essential_matrix))
         *   state \f$ E = R * [T]_x \f$, but we need the transposed \f$ R^\top \f$ for consistency with our conventions.
         * - Compute the Fundamental matrix, \f[ {F} = (K_{own}^{-1})^\top * E * K_{other}^{-1} \f]
         * - Compute the epipolar line, \f[ line = F * pix_{other} \f]
         *
         * The epipolar constraint error can be computed by measuring the distance from
         * the "own" measured pixel to this expected line.
         * The computeError() function is therefore also overloaded.
         */
        void computeExpectation();

        /**
         * \brief Compute the epipolar constraint error.
         *
         * Compute the epipolar constraint error. This corresponds to the signed distance from the epipolar line
         * to the measured pixel.
         *
         * The formula for this distance is taken from SOLA-IJCV-11, eq. (64).
         *
         * The algorithm for computing this signed distance is as follows:
         * - Given \a line, the epipolar line computed in computeExpectation().
         * - Given \a pix, the pixel measurement in the up node feature.
         * - compute the signed distance, that is,
         *      \f[ d = pix^\top * line / \sqrt{line_0^2 + line_1^2}\f]
         *
         * As such, this algorithm is a normalized version (into the pixel units) of the error in the epipolar
         * constraint \f$0 = pix^\top * {F} * pix_{other}\f$, as it can be seen by substituting the result given by computeExpectation(), \f$ line = F * pix_{other} \f$:
         *      \f[ d = pix^\top * {F} * pix_{other} / \sqrt{line_0^2 + line_1^2}\f]
         * where \f$F\f$ is the fundamental matrix between the two cameras, and the pixels are in homogeneous coordinates.
         */
        void computeError();
        /**
         * \brief Compute the epipolar constraint error.
         *
         * Compute the epipolar constraint error and place it at the given \a _residuals,
         * starting at position \a _idx and spanning the dimension of the error of this node.
         *
         * \sa computeError() for more information.
         */
        void computeError(Eigen::VectorXs & _residuals, unsigned int & _idx);
        /**
         * \brief Compute the epipolar constraint error.
         *
         * Compute the epipolar constraint error and place it at the given \a _residuals Map<VectorXs>,
         * starting at position \a _idx and spanning the dimension of the error of this node.
         *
         * \sa computeError() for more information.
         */
        void computeError(Eigen::Map<Eigen::VectorXs> & _residuals, unsigned int & _idx);

        /**
         * \brief Compute the epipolar constraint error.
         *
         * Compute the epipolar constraint error and place it at the given \a _residuals pointer,
         * plus \a _idx, and spanning the dimension of the error of this node.
         *
         * \sa computeError() for more information.
         */
void computeError(WolfScalar * _residuals, unsigned int & _idx);


};

//////////////////////////////////////
// IMPLEMENTATION
//////////////////////////////////////

#include "wolf_tools.h"
#include "pin_hole.h"

EpipolarConstraint::EpipolarConstraint(const FeatureShPtr& _ft_ptr,
                                       const FeatureShPtr& _ft_other_ptr) :
        CorrespondenceRelative(_ft_ptr, _ft_other_ptr, DIM_ERROR_, DIM_EXPECTATION_)
{
    //
}

void EpipolarConstraint::computeExpectation()
{

    // Test if we have TransSensor to the other Capture and get it by its node ID.
    auto trans_sensor_ptr = capturePtr()->transSensorPtr( captureOtherPtr()->nodeId() );

    if (trans_sensor_ptr != NULL)  // TRANS-SENSOR FOUND: USE ALL PRE-COMPUTED DATA
    {
        // Delegate all the work to the TransSensor
        trans_sensor_ptr->computeExpectation(featureOtherPtr()->measurement(), expectation_);
    }
    else  // NO TRANS-SENSOR FOUND: RE-COMPUTE RELATIVE POSE AND FUNDAMENTAL MATRIX.
    {

        using namespace Eigen;

        // NOTE: We use a temporary "Matrix3s M_tmp;"  that will be used several times

        // 1. Start with the poses stuff:

        // 1a. Composition of the relative pose - this is because we do not have trans-sensor, so we need to compute them each time.
        // We make use of pre-computed global poses in Capture.
        // M_tmp is R_rel, the relative rotation matrix:
        Matrix3s M_tmp = ( captureOtherPtr()->inverseGlobalPose().q() * capturePtr()->globalPose().q() ).matrix();
        Vector3s T_rel = captureOtherPtr()->inverseGlobalPose().q() * capturePtr()->globalPose().p()
                + captureOtherPtr()->inverseGlobalPose().p();

        // 2. Now on to the vision stuff...

        // 2a. Cast SensorBase to PinHole to access the inverse intrinsic matrices, pre-computed in PinHole at construction time.
        // NOTE: A dynamic cast is safer but here we vote for speed.
        PinHole* pin_hole_own_ptr_(static_cast<PinHolePtr>(sensorPtr() ) );
        PinHole* pin_hole_other_ptr_(static_cast<PinHolePtr>(sensorOtherPtr() ) );

        // 2b. Compute essential and fundamental matrices - this is because we do not have trans-sensor, so we need to compute them each time.
        // helper: E = R_rel' * [T_rel]_x
        // helper: F = (K_own^-1)' * E * K_other^-1
        // M_tmp becomes E, the essential matrix:
        M_tmp = M_tmp.transpose() * Wolf::skew(T_rel);
        // M_tmp becomes F, the fundamental matrix:
        M_tmp = pin_hole_own_ptr_->inverseIntrinsicMatrix().transpose() * M_tmp * pin_hole_other_ptr_->inverseIntrinsicMatrix();

        // 2c. compute the epipolar line
        // This is equivalent to epi_line = F * euclideanToHomogeneous(pix_other);
        expectation_ = M_tmp.leftCols(2) * featureOtherPtr()->measurement() + M_tmp.rightCols(1);
    }
}


inline void EpipolarConstraint::computeError()
{
    using namespace std;
    using namespace Eigen;
    computeExpectation();

    // now expectaion_ is the epipolar line
    // and featurePtr()->measurement() is the own pixel measurement of this feature, the measured point
    // the error is the distance from the point to the line.

    error_(0) = Wolf::distPointToLine( featurePtr()->measurement(), expectation() ); // distance from point to line, this is the error!

}

inline void EpipolarConstraint::computeError(Eigen::VectorXs & _residuals, unsigned int & _idx)
{
    computeExpectation();
    _residuals.segment(_idx, DIM_ERROR_)(0) = Wolf::distPointToLine( featurePtr()->measurement(), expectation() );
    _idx += DIM_ERROR_;
}


inline void EpipolarConstraint::computeError(Eigen::Map<Eigen::VectorXs> & _residuals, unsigned int & _idx)
{
    computeExpectation();
    _residuals.segment(_idx, DIM_ERROR_)(0) = Wolf::distPointToLine( featurePtr()->measurement(), expectation() );
    _idx += DIM_ERROR_;
}

inline void EpipolarConstraint::computeError(WolfScalar * _residuals, unsigned int & _idx)
{
    computeExpectation();
    *(_residuals + _idx) = Wolf::distPointToLine( featurePtr()->measurement(), expectation() );
    _idx += DIM_ERROR_;
}


#endif /* EPIPOLAR_CONSTRAINT_H_ */
