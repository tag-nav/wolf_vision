
#ifndef PROCESSOR_IMU_UNITTESTER_H
#define PROCESSOR_IMU_UNITTESTER_H

// Wolf
#include "processor_imu.h"
#include "processor_motion.h"
#include "frame_imu.h"

namespace wolf {
    struct IMU_jac_bias{ //struct used for checking jacobians by finite difference

        IMU_jac_bias(Eigen::Matrix3s _dDp_dab, Eigen::Matrix3s _dDv_dab, Eigen::Matrix3s _dDp_dwb, Eigen::Matrix3s _dDv_dwb, Eigen::Matrix3s _dDq_dwb, 
                       Eigen::Matrix<Eigen::VectorXs,6,1> _Deltas_noisy_vect, Eigen::VectorXs _Delta0 ) : dDp_dab_(_dDp_dab), 
                       dDv_dab_(_dDv_dab), dDp_dwb_(_dDp_dwb), dDv_dwb_(_dDv_dwb), dDq_dwb_(_dDq_dwb), 
                       Deltas_noisy_vect_(_Deltas_noisy_vect), Delta0_(_Delta0) {}

        public:
            /*The following vectors will contain all the matrices and deltas needed to compute the finite differences.
              place 1 : added da_bx in data         place 2 : added da_by in data       place 3 : added da_bz in data
              place 4 : added dw_bx in data         place 5 : added dw_by in data       place 6 : added dw_bz in data
             */
            Eigen::Matrix<Eigen::VectorXs,6,1> Deltas_noisy_vect_;
            Eigen::VectorXs Delta0_;
            Eigen::Matrix3s dDp_dab_;
            Eigen::Matrix3s dDv_dab_;
            Eigen::Matrix3s dDp_dwb_;
            Eigen::Matrix3s dDv_dwb_;
            Eigen::Matrix3s dDq_dwb_;
    };

    struct IMU_jac_deltas{

        IMU_jac_deltas(Eigen::VectorXs _Delta0, Eigen::VectorXs _delta0, Eigen::Matrix<Eigen::VectorXs,9,1> _Delta_noisy_vect, Eigen::Matrix<Eigen::VectorXs,9,1> _delta_noisy_vect, 
                        Eigen::MatrixXs _jacobian_delta_preint, Eigen::MatrixXs _jacobian_delta ) :
                        Delta0_(_Delta0), delta0_(_delta0), Delta_noisy_vect_(_Delta_noisy_vect), delta_noisy_vect_(_delta_noisy_vect), 
                       jacobian_delta_preint_(_jacobian_delta_preint), jacobian_delta_(_jacobian_delta) {}
        
        public:
            /*The following vectors will contain all the matrices and deltas needed to compute the finite differences.
              Elements at place 0 are those not affected by the bias noise that we add (Delta_noise, delta_noise -> dPx, dpx, dVx, dvx,..., dOz,doz).
                            Delta_noisy_vect_                                                                       delta_noisy_vect_
                            0: + 0,                                                                                 0: + 0
                            1: +dPx, 2: +dPy, 3: +dPz                                                               1: + dpx, 2: +dpy, 3: +dpz
                            4: +dVx, 5: +dVy, 6: +dVz                                                               4: + dvx, 5: +dvy, 6: +dvz
                            7: +dOx, 8: +dOy, 9: +dOz                                                               7: + dox, 8: +doy, 9: +doz
             */
            Eigen::VectorXs Delta0_; //this will contain the Delta not affected by noise
            Eigen::VectorXs delta0_; //this will contain the delta not affected by noise
            Eigen::Matrix<Eigen::VectorXs,9,1> Delta_noisy_vect_; //this will contain the Deltas affected by noises
            Eigen::Matrix<Eigen::VectorXs,9,1> delta_noisy_vect_; //this will contain the deltas affected by noises
            Eigen::MatrixXs jacobian_delta_preint_;
            Eigen::MatrixXs jacobian_delta_;
    };

    class ProcessorIMU_UnitTester : public ProcessorIMU{

        public:

        ProcessorIMU_UnitTester();
        virtual ~ProcessorIMU_UnitTester();

        //Functions to test jacobians with finite difference method

        /* params :
            _data : input data vector (size 6 : ax,ay,az,wx,wy,wz)
            _dt : time interval between 2 IMU measurements
            da_b : bias noise to add - scalar because adding the same noise to each component of bias (abx, aby, abz, wbx, wby, wbz) one by one. 
         */
        IMU_jac_bias finite_diff_ab(const Scalar _dt, Eigen::Vector6s& _data, const wolf::Scalar& da_b);

        /* params :
            _data : input data vector (size 6 : ax,ay,az,wx,wy,wz)
            _dt : time interval between 2 IMU measurements
            _Delta_noise : noise to add to Delta_preint (D1 in D = D1 + d), vector 9 because rotation expressed as a vector (R2v(q.matrix()))
            _delta_noise : noise to add to instantaneous delta (d in D = D1 + d), vector 9 because rotation expressed as a vector (R2v(q.matrix()))
         */
        IMU_jac_deltas finite_diff_noise(const Scalar& _dt, Eigen::Vector6s& _data, const Eigen::Matrix<wolf::Scalar,9,1>& _Delta_noise, const Eigen::Matrix<wolf::Scalar,9,1>& _delta_noise);

        public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
    };

}

/////////////////////////////////////////////////////////
// IMPLEMENTATION. Put your implementation includes here
/////////////////////////////////////////////////////////

// Wolf
#include "state_block.h"
#include "rotations.h"


namespace wolf{

    //Functions to test jacobians with finite difference method
inline IMU_jac_bias ProcessorIMU_UnitTester::finite_diff_ab(const Scalar _dt, Eigen::Vector6s& _data, const wolf::Scalar& da_b)
{
    //TODO : need to use a reset function here to make sure jacobians have not been used before --> reset everything
    ///Define all the needed variables
    Eigen::VectorXs Delta0;
    Eigen::Matrix<Eigen::VectorXs,6,1> Deltas_noisy_vect;
    Eigen::Vector6s data0;
    data0 = _data;

    Eigen::MatrixXs data_cov;
    Eigen::MatrixXs jacobian_delta_preint;
    Eigen::MatrixXs jacobian_delta;
    Eigen::VectorXs delta_preint_plus_delta0;
    Eigen::VectorXs delta_preint0;
    data_cov.resize(6,6);
    jacobian_delta_preint.resize(9,9);
    jacobian_delta.resize(9,9);
    delta_preint_plus_delta0.resize(10);
    delta_preint0.resize(10);

    //set all variables to 0
    data_cov = Eigen::MatrixXs::Zero(6,6);
    jacobian_delta_preint = Eigen::MatrixXs::Zero(9,9);
    jacobian_delta = Eigen::MatrixXs::Zero(9,9);
    delta_preint_plus_delta0 << 0,0,0, 0,0,0, 1,0,0,0;
    delta_preint0 << 0,0,0, 0,0,0, 1,0,0,0;

    /*The following vectors will contain all the matrices and deltas needed to compute the finite differences.
        place 1 : added da_bx in data         place 2 : added da_by in data       place 3 : added da_bz in data
        place 4 : added dw_bx in data         place 5 : added dw_by in data       place 6 : added dw_bz in data
     */

    Eigen::Matrix3s dDp_dab, dDv_dab, dDp_dwb, dDv_dwb, dDq_dwb;

    //Deltas without use of da_b
    data2delta(_data, data_cov, _dt);
    deltaPlusDelta(delta_preint0, delta_, _dt, delta_preint_plus_delta0, jacobian_delta_preint, jacobian_delta);
    Delta0 = delta_preint_plus_delta0; //this is the first preintegrated delta, not affected by any added bias noise
    dDp_dab = dDp_dab_;
    dDv_dab = dDv_dab_;
    dDp_dwb = dDp_dwb_;
    dDv_dwb = dDv_dwb_;
    dDq_dwb = dDq_dwb_;

    // propagate bias noise
    for(int i=0; i<6; i++){
        //need to reset stuff here
        acc_bias_ = Eigen::Vector3s::Zero();
        gyro_bias_ = Eigen::Vector3s::Zero();
        dDp_dab_.setZero();
        dDv_dab_.setZero();
        dDp_dwb_.setZero();
        dDv_dwb_.setZero();
        dDq_dwb_.setZero();
        delta_preint_plus_delta0 << 0,0,0, 0,0,0, 1,0,0,0;
        delta_preint0 << 0,0,0, 0,0,0, 1,0,0,0;
        data_cov = Eigen::MatrixXs::Zero(6,6);

        // add da_b
        _data = data0;
        _data(i) = _data(i) - da_b; //- because a = a_m − a_b + a_n, in out case, a = a_m − a_b - da_b + a_n
        //data2delta
        data2delta(_data, data_cov, _dt);
        deltaPlusDelta(delta_preint0, delta_, _dt, delta_preint_plus_delta0, jacobian_delta_preint, jacobian_delta);
        Deltas_noisy_vect(i) = delta_preint_plus_delta0; //preintegrated deltas affected by added bias noise
    }

    IMU_jac_bias bias_jacobians(dDp_dab, dDv_dab, dDp_dwb, dDv_dwb, dDq_dwb, Deltas_noisy_vect, Delta0);
    return bias_jacobians;
}

inline IMU_jac_deltas ProcessorIMU_UnitTester::finite_diff_noise(const Scalar& _dt, Eigen::Vector6s& _data, const Eigen::Matrix<wolf::Scalar,9,1>& _Delta_noise, const Eigen::Matrix<wolf::Scalar,9,1>& _delta_noise)
{
    //we do not propagate any noise from data vector
    Eigen::VectorXs Delta0;
    Eigen::VectorXs Delta_;
    Eigen::VectorXs delta0;
    Eigen::VectorXs delta_preint_plus_delta;
    Delta0.resize(10);
    delta0.resize(10);
    delta_preint_plus_delta.resize(10);
    Delta0.setZero();
    delta_preint_plus_delta.setZero();
    Eigen::MatrixXs jacobian_delta_preint;
    Eigen::MatrixXs jacobian_delta;
    jacobian_delta_preint.resize(9,9);
    jacobian_delta.resize(9,9);
    jacobian_delta_preint = Eigen::MatrixXs::Zero(9,9);
    jacobian_delta = Eigen::MatrixXs::Zero(9,9);
    Eigen::MatrixXs jacobian_delta_preint0;
    Eigen::MatrixXs jacobian_delta0;
    jacobian_delta_preint0.setZero();
    jacobian_delta0.setZero();;

    Eigen::MatrixXs data_cov;
    data_cov.resize(6,6);
    data_cov = Eigen::MatrixXs::Zero(6,6);

    Eigen::Matrix<Eigen::VectorXs,9,1> Delta_noisy_vect; //this will contain the Deltas affected by noises
    Eigen::Matrix<Eigen::VectorXs,9,1> delta_noisy_vect; //this will contain the deltas affected by noises

    data2delta(_data, data_cov, _dt); //Affects dp_out, dv_out and dq_out
    deltaPlusDelta(Delta0, delta0, _dt, delta_preint_plus_delta, jacobian_delta_preint, jacobian_delta); 
    jacobian_delta_preint0 = jacobian_delta_preint;
    jacobian_delta0 = jacobian_delta;

    //We compute all the jacobians wrt deltas and noisy deltas
    for(int i=0; i<6; i++) //for 6 first component we just add to add noise as vector component since it is in the R^3 space
    {
        //fist we need to reset some stuff
        jacobian_delta_preint.setZero();
        jacobian_delta.setZero();
        acc_bias_.setZero();
        gyro_bias_.setZero();

        delta_ = delta0;
        delta_(i) = delta_(i) + _delta_noise(i); //noise has been added
        deltaPlusDelta(Delta0, delta_, _dt, delta_preint_plus_delta, jacobian_delta_preint, jacobian_delta);
        delta_noisy_vect(i) = delta_;
    }

    for(int i=0; i<3; i++) //for noise dtheta, it is in SO3, need to work on quaternions
    {
        //fist we need to reset some stuff
        Eigen::Matrix3s dqr_tmp;
        Eigen::Vector3s dtheta = Eigen::Vector3s::Zero();
        jacobian_delta_preint.setZero();
        jacobian_delta.setZero();
        acc_bias_.setZero();
        gyro_bias_.setZero();

        delta_ = delta0;
        remapDelta(delta_); //not sure that we need this
        dqr_tmp = Dq_out_.matrix();
        dtheta(i) +=  _delta_noise(i+6); //introduce perturbation
        dqr_tmp = dqr_tmp * v2R(dtheta); //Apply perturbation : R * exp(dtheta) --> using matrix
        Dq_out_ = v2q(R2v(dqr_tmp)); //orientation noise has been added --> get back to quaternion form
        deltaPlusDelta(Delta0, delta_, _dt, delta_preint_plus_delta, jacobian_delta_preint, jacobian_delta);
        delta_noisy_vect(i+6) = delta_;
    }

    //We compute all the jacobians wrt Deltas and noisy Deltas
    for(int i=0; i<6; i++) //for 6 first component we just add to add noise as vector component since it is in the R^3 space
    {
        //fist we need to reset some stuff
        jacobian_delta_preint.setZero();
        jacobian_delta.setZero();
        acc_bias_.setZero();
        gyro_bias_.setZero();

        Delta_ = Delta0;
        Delta_(i) = Delta_(i) + _Delta_noise(i); //noise has been added
        deltaPlusDelta(Delta_, delta0, _dt, delta_preint_plus_delta, jacobian_delta_preint, jacobian_delta);
        Delta_noisy_vect(i) = Delta_;
    }

    for(int i=0; i<3; i++) //for noise dtheta, it is in SO3, need to work on quaternions
    {
        //fist we need to reset some stuff
        Eigen::Matrix3s dQr_tmp;
        Eigen::Vector3s dtheta = Eigen::Vector3s::Zero();
        jacobian_delta_preint.setZero();
        jacobian_delta.setZero();
        acc_bias_.setZero();
        gyro_bias_.setZero();

        Delta_ = Delta0;
        remapDelta(Delta_); //this time we need it
        dQr_tmp = Dq_out_.matrix();
        dtheta(i) += _Delta_noise(i+6); //introduce perturbation
        dQr_tmp = dQr_tmp * v2R(dtheta); //Apply perturbation : R * exp(dtheta) --> using matrix
        Dq_out_ = v2q(R2v(dQr_tmp)); //orientation noise has been added --> get back to quaternion form
        deltaPlusDelta(Delta_, delta0, _dt, delta_preint_plus_delta, jacobian_delta_preint, jacobian_delta);
        Delta_noisy_vect(i+6) = Delta_;
    }
    
    IMU_jac_deltas jac_deltas(Delta0, delta0, Delta_noisy_vect, delta_noisy_vect, jacobian_delta_preint0, jacobian_delta0);
    return jac_deltas;

}

} // namespace wolf

#endif // PROCESSOR_IMU_UNITTESTER_H
