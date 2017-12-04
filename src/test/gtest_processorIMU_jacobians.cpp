/**
 * \file gtest_imu_preintegration_jacobians.cpp
 *
 *  Created on: Nov 29, 2016
 *      \author: AtDinesh
 */

 //Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu_UnitTester.h"

//std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>

//google test
#include "utils_gtest.h"

//#define DEBUG_RESULTS
//#define WRITE_RESULTS

using namespace wolf;

// A new one of these is created for each test
class ProcessorIMU_jacobians : public testing::Test
{
    public:
        TimeStamp t;
        Eigen::Vector6s data_;
        Eigen::Matrix<wolf::Scalar,10,1> Delta0;
        wolf::Scalar ddelta_bias;
        wolf::Scalar dt;
        Eigen::Matrix<wolf::Scalar,9,1> Delta_noise;
        Eigen::Matrix<wolf::Scalar,9,1> delta_noise;
        struct IMU_jac_bias bias_jac;
        struct IMU_jac_deltas deltas_jac;

        void remapJacDeltas_quat0(IMU_jac_deltas& _jac_delta, Eigen::Map<Eigen::Quaternions>& _Dq0, Eigen::Map<Eigen::Quaternions>& _dq0){

            new (&_Dq0) Eigen::Map<const Eigen::Quaternions>(_jac_delta.Delta0_.data() + 3);
            new (&_dq0) Eigen::Map<const Eigen::Quaternions>(_jac_delta.delta0_.data() + 3);
        }

        void remapJacDeltas_quat(IMU_jac_deltas& _jac_delta, Eigen::Map<Eigen::Quaternions>& _Dq, Eigen::Map<Eigen::Quaternions>& _dq, const int& place ){
    
            assert(place < _jac_delta.Delta_noisy_vect_.size());
            new (&_Dq) Eigen::Map<const Eigen::Quaternions>(_jac_delta.Delta_noisy_vect_(place).data() + 3);
            new (&_dq) Eigen::Map<const Eigen::Quaternions>(_jac_delta.delta_noisy_vect_(place).data() + 3);
        }

    virtual void SetUp()
    {
        //SetUp for jacobians 
        wolf::Scalar deg_to_rad = M_PI/180.0;
        data_ << 10,0.5,3, 100*deg_to_rad,110*deg_to_rad,30*deg_to_rad;

        // Wolf problem
        ProblemPtr wolf_problem_ptr_ = Problem::create("POV 3D");
        Eigen::VectorXs IMU_extrinsics(7);
        IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot

        ProcessorIMU_UnitTester processor_imu;
        ddelta_bias = 0.00000001;
        dt = 0.001;

        //defining a random Delta to begin with (not to use Origin point)
        Eigen::Matrix<wolf::Scalar,10,1> Delta0;
        Delta0 = Eigen::Matrix<wolf::Scalar,10,1>::Random();
        Delta0.head<3>() = Delta0.head<3>()*100;
        Delta0.tail<3>() = Delta0.tail<3>()*10;
        Eigen::Vector3s ang0, ang;
        ang0 << 120.08*deg_to_rad, 12.36*deg_to_rad, 54.32*deg_to_rad; 

        Eigen::Map<Eigen::Quaternions> Delta0_quat(Delta0.data()+3);
        Delta0_quat = v2q(ang0);
        Delta0_quat.normalize();
        ang = q2v(Delta0_quat);

        //std::cout << "\ninput Delta0 : " << Delta0 << std::endl;
        //std::cout << "\n rotation vector we start with :\n" << ang << std::endl;

        //get data to compute jacobians
        struct IMU_jac_bias bias_jac_c = processor_imu.finite_diff_ab(dt, data_, ddelta_bias, Delta0);
        bias_jac.copyfrom(bias_jac_c);

        Delta_noise << 0.00000001, 0.00000001, 0.00000001,  0.0001, 0.0001, 0.0001,  0.00000001, 0.00000001, 0.00000001;
        delta_noise << 0.00000001, 0.00000001, 0.00000001,  0.0001, 0.0001, 0.0001,  0.00000001, 0.00000001, 0.00000001;

        struct IMU_jac_deltas deltas_jac_c = processor_imu.finite_diff_noise(dt, data_, Delta_noise, delta_noise, Delta0);
        deltas_jac = deltas_jac_c;
    }

    virtual void TearDown()
    {
        // code here will be called just after the test completes
        // ok to through exceptions from here if need be
        /*
            You can do deallocation of resources in TearDown or the destructor routine. 
                However, if you want exception handling you must do it only in the TearDown code because throwing an exception 
                from the destructor results in undefined behavior.
            The Google assertion macros may throw exceptions in platforms where they are enabled in future releases. 
                Therefore, it's a good idea to use assertion macros in the TearDown code for better maintenance.
        */
    }
};


class ProcessorIMU_jacobians_Dq : public testing::Test
{
    public:
        TimeStamp t;
        Eigen::Vector6s data_;
        Eigen::Matrix<wolf::Scalar,10,1> Delta0;
        wolf::Scalar ddelta_bias2;
        wolf::Scalar dt;
        struct IMU_jac_bias bias_jac2;

        void remapJacDeltas_quat0(IMU_jac_deltas& _jac_delta, Eigen::Map<Eigen::Quaternions>& _Dq0, Eigen::Map<Eigen::Quaternions>& _dq0){

            new (&_Dq0) Eigen::Map<const Eigen::Quaternions>(_jac_delta.Delta0_.data() + 3);
            new (&_dq0) Eigen::Map<const Eigen::Quaternions>(_jac_delta.delta0_.data() + 3);
        }

        void remapJacDeltas_quat(IMU_jac_deltas& _jac_delta, Eigen::Map<Eigen::Quaternions>& _Dq, Eigen::Map<Eigen::Quaternions>& _dq, const int& place ){
    
            assert(place < _jac_delta.Delta_noisy_vect_.size());
            new (&_Dq) Eigen::Map<const Eigen::Quaternions>(_jac_delta.Delta_noisy_vect_(place).data() + 3);
            new (&_dq) Eigen::Map<const Eigen::Quaternions>(_jac_delta.delta_noisy_vect_(place).data() + 3);
        }

    virtual void SetUp()
    {
        //SetUp for jacobians 
        wolf::Scalar deg_to_rad = M_PI/180.0;
        data_ << 10,0.5,3, 100*deg_to_rad,110*deg_to_rad,30*deg_to_rad;

        // Wolf problem
        ProblemPtr wolf_problem_ptr_ = Problem::create("POV 3D");
        Eigen::VectorXs IMU_extrinsics(7);
        IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot

        ProcessorIMU_UnitTester processor_imu;
        ddelta_bias2 = 0.0001;
        dt = 0.001;

        //defining a random Delta to begin with (not to use Origin point)
        Eigen::Matrix<wolf::Scalar,10,1> Delta0;
        Delta0 = Eigen::Matrix<wolf::Scalar,10,1>::Random();
        Delta0.head<3>() = Delta0.head<3>()*100;
        Delta0.tail<3>() = Delta0.tail<3>()*10;
        Eigen::Vector3s ang0, ang;
        ang0 << 120.08*deg_to_rad, 12.36*deg_to_rad, 54.32*deg_to_rad; 

        Eigen::Map<Eigen::Quaternions> Delta0_quat(Delta0.data()+3);
        Delta0_quat = v2q(ang0);
        Delta0_quat.normalize();
        ang = q2v(Delta0_quat);

        //std::cout << "\ninput Delta0 : " << Delta0 << std::endl;
        //std::cout << "\n rotation vector we start with :\n" << ang << std::endl;

        //get data to compute jacobians
        struct IMU_jac_bias bias_jac_c = processor_imu.finite_diff_ab(dt, data_, ddelta_bias2, Delta0);
        bias_jac2.copyfrom(bias_jac_c);
    }

    virtual void TearDown()
    {
        // code here will be called just after the test completes
        // ok to through exceptions from here if need be
        /*
            You can do deallocation of resources in TearDown or the destructor routine. 
                However, if you want exception handling you must do it only in the TearDown code because throwing an exception 
                from the destructor results in undefined behavior.
            The Google assertion macros may throw exceptions in platforms where they are enabled in future releases. 
                Therefore, it's a good idea to use assertion macros in the TearDown code for better maintenance.
        */
    }
};

                            ///BIAS TESTS

/*                              IMU_jac_deltas struct form :
    contains vectors of size 7 :
    Elements at place 0 are those not affected by the bias noise that we add (da_bx,..., dw_bx,... ).
                place 1 : added da_bx in data         place 2 : added da_by in data       place 3 : added da_bz in data
                place 4 : added dw_bx in data         place 5 : added dw_by in data       place 6 : added dw_bz in data
*/

/*                                  Mathematics

        dDp_dab = [dDp_dab_x, dDp_dab_y, dDp_dab_z]
        dDp_dab_x = (dDp(ab_x + dab_x, ab_y, ab_z) - dDp(ab_x,ab_y,ab_z)) / dab_x
        dDp_dab_x = (dDp(ab_x, ab_y + dab_y, ab_z) - dDp(ab_x,ab_y,ab_z)) / dab_y
        dDp_dab_x = (dDp(ab_x, ab_y, ab_z + dab_z) - dDp(ab_x,ab_y,ab_z)) / dab_z

        similar for dDv_dab
        note dDp_dab_x, dDp_dab_y, dDp_dab_z, dDv_dab_x, dDv_dab_y, dDv_dab_z are 3x1 vectors !

        dDq_dab = 0_{3x3}
        dDq_dwb = [dDq_dwb_x, dDq_dwb_y, dDq_dwb_z]
        dDq_dwb_x = log( dR(wb).transpose() * dR(wb - dwb_x))/dwb_x
                  = log( dR(wb).transpose() * exp((wx - wbx - dwb_x)dt, (wy - wby)dt, (wy - wby)dt))/dwb_x
        dDq_dwb_y = log( dR(wb).transpose() * dR(wb - dwb_y))/dwb_y
        dDq_dwb_z = log( dR(wb).transpose() * dR(wb + dwb_z))/dwb_z

        Note : dDq_dwb must be computed recursively ! So comparing the one returned by processor_imu and the numerical 
        one will have no sense if we aredoing this from a random Delta. The Delta here should be the origin.
        dDq_dwb_ = dR.tr()*dDq_dwb - Jr(wdt)*dt
        Then at first step, dR.tr() = Id, dDq_dwb = 0_{3x3}, which boils down to dDq_dwb_ = Jr(wdt)*dt 
*/

TEST_F(ProcessorIMU_jacobians, dDp_dab)
{
    using namespace wolf;
    Eigen::Matrix3s dDp_dab;

    for(int i=0;i<3;i++)
         dDp_dab.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i).head(3) - bias_jac.Delta0_.head(3))/ddelta_bias;

    ASSERT_TRUE((dDp_dab - bias_jac.dDp_dab_).isMuchSmallerThan(1,0.000001)) << "dDp_dab : \n" << dDp_dab << "\n bias_jac.dDp_dab_ :\n" << bias_jac.dDp_dab_ <<
     "\ndDp_dab_a - dDp_dab_ : \n" << bias_jac.dDp_dab_ - dDp_dab << std::endl;
}

TEST_F(ProcessorIMU_jacobians, dDv_dab)
{
    using namespace wolf;
    Eigen::Matrix3s dDv_dab;

    for(int i=0;i<3;i++)
         dDv_dab.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i).tail(3) - bias_jac.Delta0_.tail(3))/ddelta_bias;

    ASSERT_TRUE((dDv_dab - bias_jac.dDv_dab_).isMuchSmallerThan(1,0.000001)) << "dDv_dab : \n" << dDv_dab << "\n bias_jac.dDv_dab_ :\n" << bias_jac.dDv_dab_ <<
     "\ndDv_dab_a - dDv_dab_ : \n" << bias_jac.dDv_dab_ - dDv_dab << std::endl;
}

TEST_F(ProcessorIMU_jacobians, dDp_dwb)
{
    using namespace wolf;
    Eigen::Matrix3s dDp_dwb;

    for(int i=0;i<3;i++)
         dDp_dwb.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i+3).head(3) - bias_jac.Delta0_.head(3))/ddelta_bias;

    ASSERT_TRUE((dDp_dwb - bias_jac.dDp_dwb_).isMuchSmallerThan(1,0.000001)) << "dDp_dwb : \n" << dDp_dwb << "\n bias_jac.dDp_dwb_ :\n" << bias_jac.dDp_dwb_ <<
     "\ndDp_dwb_a - dDv_dab_ : \n" << bias_jac.dDp_dwb_ - dDp_dwb << std::endl;
}

TEST_F(ProcessorIMU_jacobians, dDv_dwb_)
{
    using namespace wolf;
    Eigen::Matrix3s dDv_dwb;

    for(int i=0;i<3;i++)
         dDv_dwb.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i+3).tail(3) - bias_jac.Delta0_.tail(3))/ddelta_bias;

    ASSERT_TRUE((dDv_dwb - bias_jac.dDv_dwb_).isMuchSmallerThan(1,0.000001)) << "dDv_dwb : \n" << dDv_dwb << "\n bias_jac.dDv_dwb_ :\n" << bias_jac.dDv_dwb_ <<
     "\ndDv_dwb_a - dDv_dwb_ : \n" << bias_jac.dDv_dwb_ - dDv_dwb << std::endl;
}

TEST_F(ProcessorIMU_jacobians, dDq_dab)
{
    using namespace wolf;
    Eigen::Map<Eigen::Quaternions> q_in_1(NULL), q_in_2(NULL);
    Eigen::Matrix3s dDq_dab;

    new (&q_in_1) Eigen::Map<Eigen::Quaternions>(bias_jac.Delta0_.data() + 3);
    for(int i=0;i<3;i++){
        new (&q_in_2) Eigen::Map<Eigen::Quaternions>(bias_jac.Deltas_noisy_vect_(i).data() + 3);
        dDq_dab.block<3,1>(0,i) = R2v( q_in_1.matrix().transpose() * q_in_2.matrix())/ddelta_bias;
    }

    ASSERT_TRUE(dDq_dab.isZero(wolf::Constants::EPS)) << "\t\tdDq_dab_ jacobian is not Zero :" << dDq_dab << std::endl;
}

TEST_F(ProcessorIMU_jacobians, dDq_dwb_noise_1Em8_)
{
    using namespace wolf;
    Eigen::Map<Eigen::Quaternions> q_in_1(NULL), q_in_2(NULL);
    Eigen::Matrix3s dDq_dwb;

    new (&q_in_1) Eigen::Map<Eigen::Quaternions>(bias_jac.Delta0_.data() + 3);

    for(int i=0;i<3;i++){
        new (&q_in_2) Eigen::Map<Eigen::Quaternions>(bias_jac.Deltas_noisy_vect_(i+3).data() + 3);

        dDq_dwb.block<3,1>(0,i) = R2v( q_in_1.matrix().transpose() * q_in_2.matrix())/ddelta_bias;
    }

    ASSERT_TRUE((dDq_dwb - bias_jac.dDq_dwb_).isMuchSmallerThan(1,0.000001))
        << "dDq_dwb : \n" << dDq_dwb
        << "\n bias_jac.dDq_dwb_ :\n" << bias_jac.dDq_dwb_
        << "\ndDq_dwb_a - dDq_dwb_ : \n" << bias_jac.dDq_dwb_ - dDq_dwb
        << "\n R1^T * R2 : \n"  << q_in_1.matrix().transpose() * q_in_2.matrix()
        << std::endl;
}

TEST_F(ProcessorIMU_jacobians_Dq, dDq_dwb_noise_1Em4_precision_1Em3_)
{
    using namespace wolf;
    Eigen::Map<Eigen::Quaternions> q_in_1(NULL), q_in_2(NULL);
    Eigen::Matrix3s dDq_dwb;

    new (&q_in_1) Eigen::Map<Eigen::Quaternions>(bias_jac2.Delta0_.data() + 3);
    for(int i=0;i<3;i++){
        new (&q_in_2) Eigen::Map<Eigen::Quaternions>(bias_jac2.Deltas_noisy_vect_(i+3).data() + 3);
        dDq_dwb.block<3,1>(0,i) = R2v( q_in_1.matrix().transpose() * q_in_2.matrix())/ddelta_bias2;
    }

    ASSERT_TRUE((dDq_dwb - bias_jac2.dDq_dwb_).isMuchSmallerThan(1,0.001)) << "dDq_dwb : \n" << dDq_dwb << "\n bias_jac2.dDq_dwb_ :\n" << bias_jac2.dDq_dwb_ <<
     "\ndDq_dwb_a - dDq_dwb_ : \n" << bias_jac2.dDq_dwb_ - dDq_dwb << "\n R1^T * R2 : \n"  << q_in_1.matrix().transpose() * q_in_2.matrix() << std::endl;
}

                                ///Perturbation TESTS

/* reminder : 
                            jacobian_delta_preint_vect_                                                            jacobian_delta_vect_
                            0: + 0,                                                                                 0: + 0
                            1: +dPx, 2: +dPy, 3: +dPz                                                               1: + dpx, 2: +dpy, 3: +dpz
                            4: +dOx, 5: +dOy, 6: +dOz                                                               4: + dox, 5: +doy, 6: +doz
                            7: +dVx, 8: +dVy, 9: +dVz                                                               7: + dvx, 8: +dvy, 9: +dvz
*/

/*              Numerical method to check jacobians wrt noise

                                                            dDp_dP = [dDp_dPx, dDp_dPy, dDp_dPz]
    dDp_dPx = ((P + dPx) - P)/dPx = Id
    dDp_dPy = ((P + dPy) - P)/dPy = Id
    dDp_dPz = ((P + dPz) - P)/dPz = Id

                                                            dDp_dV = [dDp_dVx, dDp_dVy, dDp_dVz]
    dDp_dVx = ((V + dVx)*dt - V*dt)/dVx = Id*dt
    dDp_dVy = ((V + dVy)*dt - V*dt)/dVy = Id*dt
    dDp_dVz = ((V + dVz)*dt - V*dt)/dVz = Id*dt

                                                            dDp_dO = [dDp_dOx, dDp_dOy, dDp_dOz]
    dDp_dOx = (( dR(Theta + dThetax)*dp ) - ( dR(Theta)*dp ))/dThetax 
            = (( dR(Theta) * exp(dThetax,0,0)*dp ) - ( dR(Theta)*dp ))/dThetax
    dDp_dOy = (( dR(Theta) * exp(0,dThetay,0)*dp ) - ( dR(Theta)*dp ))/dThetay
    dDp_dOz = (( dR(Theta) * exp(0,0,dThetaz)*dp ) - ( dR(Theta)*dp ))/dThetaz

                                                            dDv_dP = [dDv_dPx, dDv_dPy, dDv_dPz] = [0, 0, 0]

                                                            dDv_dV = [dDv_dVx, dDv_dVy, dDv_dVz]
    dDv_dVx = ((V + dVx) - V)/dVx = Id
    dDv_dVy = ((V + dVy) - V)/dVy = Id
    dDv_dVz = ((V + dVz) - V)/dVz = Id
    
                                                            dDv_dO = [dDv_dOx, dDv_dOy, dDv_dOz]
    dDv_dOx = (( dR(Theta + dThetax)*dv ) - ( dR(Theta)*dv ))/dThetax 
            = (( dR(Theta) * exp(dThetax,0,0)*dv ) - ( dR(Theta)*dv ))/dThetax
    dDv_dOx = (( dR(Theta) * exp(0,dThetay,0)*dv ) - ( dR(Theta)*dv ))/dThetay
    dDv_dOz = (( dR(Theta) * exp(0,0,dThetaz)*dv ) - ( dR(Theta)*dv ))/dThetaz

                                                            dDp_dp = [dDp_dpx, dDp_dpy, dDp_dpz]
    dDp_dpx = ( dR*(p + dpx) - dR*(p))/dpx
    dDp_dpy = ( dR*(p + dpy) - dR*(p))/dpy
    dDp_dpz = ( dR*(p + dpz) - dR*(p))/dpy

                                                            dDp_dv = [dDp_dvx, dDp_dvy, dDp_dvz] = [0, 0, 0]
    
                                                            dDp_do = [dDp_dox, dDp_doy, dDp_doz] = [0, 0, 0]

                                                            dDv_dp = [dDv_dpx, dDv_dpy, dDv_dpz] = [0, 0, 0]
                                                            
                                                            dDv_dv = [dDv_dvx, dDv_dvy, dDv_dvz]
    dDv_dvx = ( dR*(v + dvx) - dR*(v))/dvx
    dDv_dvy = ( dR*(v + dvy) - dR*(v))/dvy
    dDv_dvz = ( dR*(v + dvz) - dR*(v))/dvz

                                                            dDv_do = [dDv_dox, dDv_doy, dDv_doz] = [0, 0, 0]

                                                            dDo_dp = dDo_dv = dDo_dP = dDo_dV = [0, 0, 0]

                                                            dDo_dO = [dDo_dOx, dDo_dOy, dDo_dOz]
                                                            
    dDo_dOx = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta+dThetax) * dr(theta) )/dThetax
            = log( (dR(Theta) * dr(theta)).transpose() * (dR(Theta)*exp(dThetax,0,0)) * dr(theta) )/dThetax
            = log( (_Delta * _delta).transpose() * (_Delta_noisy * _delta))
    dDo_dOy = log( (dR(Theta) * dr(theta)).transpose() * (dR(Theta)*exp(0,dThetay,0)) * dr(theta) )/dThetay
    dDo_dOz = log( (dR(Theta) * dr(theta)).transpose() * (dR(Theta)*exp(0,0,dThetaz)) * dr(theta) )/dThetaz

                                                            dDo_do = [dDo_dox, dDo_doy, dDo_doz]

    dDo_dox = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta) * dr(theta+dthetax) )/dthetax
            = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta) * (dr(theta)*exp(dthetax,0,0)) )/dthetax
            = log( (_Delta * _delta).transpose() * (_Delta * _delta_noisy))
    dDo_doy = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta) * (dr(theta)*exp(0,dthetay,0)) )/dthetay
    dDo_doz = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta) * (dr(theta)*exp(0,0,dthetaz)) )/dthetaz
     */
     
TEST_F(ProcessorIMU_jacobians, dDp_dP)
{
    using namespace wolf;
    Eigen::Matrix3s dDp_dP;

    //dDp_dPx = ((P + dPx) - P)/dPx
    for(int i=0;i<3;i++)
        dDp_dP.block<3,1>(0,i) = (deltas_jac.Delta_noisy_vect_(i).head(3) - deltas_jac.Delta0_.head(3))/Delta_noise(i);

    ASSERT_TRUE((dDp_dP - deltas_jac.jacobian_delta_preint_.block(0,0,3,3)).isMuchSmallerThan(1,0.000001)) << "dDp_dP : \n" << dDp_dP << "\n deltas_jac.jacobian_delta_preint_.block(0,0,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(0,0,3,3) <<
     "\ndDp_dP_a - dDp_dP_ : \n" << deltas_jac.jacobian_delta_preint_.block(0,0,3,3) - dDp_dP << std::endl;
}

TEST_F(ProcessorIMU_jacobians, dDp_dV)
{
    using namespace wolf;
    Eigen::Matrix3s dDp_dV;

    //Dp_dVx = ((V + dVx)*dt - V*dt)/dVx
    for(int i=0;i<3;i++)
        dDp_dV.block<3,1>(0,i) = (deltas_jac.Delta_noisy_vect_(i+6).tail(3)*dt - deltas_jac.Delta0_.tail(3)*dt)/Delta_noise(i+6);

    ASSERT_TRUE((dDp_dV - deltas_jac.jacobian_delta_preint_.block(0,6,3,3)).isMuchSmallerThan(1,0.000001)) << "dDp_dV : \n" << dDp_dV << "\n deltas_jac.jacobian_delta_preint_.block(0,6,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(0,6,3,3) <<
     "\ndDp_dV_a - dDp_dV_ : \n" << deltas_jac.jacobian_delta_preint_.block(0,6,3,3) - dDp_dV << std::endl;
}

TEST_F(ProcessorIMU_jacobians, dDp_dO)
{
    using namespace wolf;
    Eigen::Map<Eigen::Quaternions> Dq0(NULL), dq0(NULL), Dq_noisy(NULL), dq_noisy(NULL);
    Eigen::Matrix3s dDp_dO;

    //dDp_dOx = (( dR(Theta) * exp(dThetax,0,0)*dp ) - ( dR(Theta)*dp ))/dThetax
    remapJacDeltas_quat0(deltas_jac, Dq0, dq0);
    for(int i=0;i<3;i++){
        remapJacDeltas_quat(deltas_jac, Dq_noisy, dq_noisy, i+3);
        dDp_dO.block<3,1>(0,i) = ((Dq_noisy.matrix() * deltas_jac.delta0_.head(3)) - (Dq0.matrix()* deltas_jac.delta0_.head(3)))/Delta_noise(i+3);
    }

    ASSERT_TRUE((dDp_dO - deltas_jac.jacobian_delta_preint_.block(0,3,3,3)).isMuchSmallerThan(1,0.000001)) << "dDp_dO : \n" << dDp_dO << "\n deltas_jac.jacobian_delta_preint_.block(0,3,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(0,3,3,3) <<
     "\ndDp_dO_a - dDp_dO_ : \n" << deltas_jac.jacobian_delta_preint_.block(0,3,3,3) - dDp_dO << std::endl;
}

TEST_F(ProcessorIMU_jacobians, dDv_dV)
{
    using namespace wolf;
    Eigen::Matrix3s dDv_dV;

    //dDv_dVx = ((V + dVx) - V)/dVx
    for(int i=0;i<3;i++)
        dDv_dV.block<3,1>(0,i) = (deltas_jac.Delta_noisy_vect_(i+6).tail(3) - deltas_jac.Delta0_.tail(3))/Delta_noise(i+6);

    ASSERT_TRUE((dDv_dV - deltas_jac.jacobian_delta_preint_.block(6,6,3,3)).isMuchSmallerThan(1,0.000001)) << "dDv_dV : \n" << dDv_dV << "\n deltas_jac.jacobian_delta_preint_.block(6,6,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(6,6,3,3) <<
     "\ndDv_dV_a - dDv_dV_ : \n" << deltas_jac.jacobian_delta_preint_.block(6,6,3,3) - dDv_dV << std::endl;
}

TEST_F(ProcessorIMU_jacobians, dDv_dO)
{
    using namespace wolf;
    Eigen::Map<Eigen::Quaternions> Dq0(NULL), dq0(NULL), Dq_noisy(NULL), dq_noisy(NULL);
    Eigen::Matrix3s dDv_dO;

    //dDv_dOx = (( dR(Theta) * exp(dThetax,0,0)*dv ) - ( dR(Theta)*dv ))/dThetax
    remapJacDeltas_quat0(deltas_jac, Dq0, dq0);
    for(int i=0;i<3;i++){
        remapJacDeltas_quat(deltas_jac, Dq_noisy, dq_noisy, i+3);
        dDv_dO.block<3,1>(0,i) = ((Dq_noisy.matrix() * deltas_jac.delta0_.tail(3)) - (Dq0.matrix()* deltas_jac.delta0_.tail(3)))/Delta_noise(i+3);
    }

    ASSERT_TRUE((dDv_dO - deltas_jac.jacobian_delta_preint_.block(6,3,3,3)).isMuchSmallerThan(1,0.000001)) << "dDv_dO : \n" << dDv_dO << "\n deltas_jac.jacobian_delta_preint_.block(6,3,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(6,3,3,3) <<
     "\ndDv_dO_a - dDv_dO_ : \n" << deltas_jac.jacobian_delta_preint_.block(6,3,3,3) - dDv_dO << std::endl;
}

//dDo_dP = dDo_dV = [0, 0, 0]


TEST_F(ProcessorIMU_jacobians, dDo_dO)
{
    using namespace wolf;
    Eigen::Map<Eigen::Quaternions> Dq0(NULL), dq0(NULL), Dq_noisy(NULL), dq_noisy(NULL);
    Eigen::Matrix3s dDo_dO;

    //dDo_dOx = log( (dR(Theta) * dr(theta)).transpose() * (dR(Theta)*exp(dThetax,0,0)) * dr(theta) )/dThetax
    remapJacDeltas_quat0(deltas_jac, Dq0, dq0);
    for(int i=0;i<3;i++){
        remapJacDeltas_quat(deltas_jac, Dq_noisy, dq_noisy, i+3);
        dDo_dO.block<3,1>(0,i) = R2v( (Dq0.matrix() * dq0.matrix()).transpose() * (Dq_noisy.matrix() * dq0.matrix()) )/Delta_noise(i+3);
    }

    ASSERT_TRUE((dDo_dO - deltas_jac.jacobian_delta_preint_.block(3,3,3,3)).isMuchSmallerThan(1,0.000001)) << "dDo_dO : \n" << dDo_dO << "\n deltas_jac.jacobian_delta_preint_.block(3,3,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(3,3,3,3) <<
     "\ndDo_dO_a - dDo_dO_ : \n" << deltas_jac.jacobian_delta_preint_.block(3,3,3,3) - dDo_dO << std::endl;
}

TEST_F(ProcessorIMU_jacobians, dDp_dp)
{
    using namespace wolf;
    Eigen::Map<Eigen::Quaternions> Dq0(NULL), dq0(NULL);
    Eigen::Matrix3s dDp_dp, dDp_dp_a;

    dDp_dp_a = deltas_jac.jacobian_delta_.block(0,0,3,3);
    remapJacDeltas_quat0(deltas_jac, Dq0, dq0);
    //dDp_dpx = ( dR*(p + dpx) - dR*(p))/dpx
    for(int i=0;i<3;i++)
        dDp_dp.block<3,1>(0,i) = ( (Dq0.matrix() * deltas_jac.delta_noisy_vect_(i).head(3)) - (Dq0.matrix() * deltas_jac.delta0_.head(3)) )/delta_noise(i);

    ASSERT_TRUE((dDp_dp - dDp_dp_a).isMuchSmallerThan(1,0.000001)) << "dDp_dp : \n" << dDp_dp << "\n dDp_dp_a :\n" << dDp_dp_a <<
     "\ndDp_dp_a - dDp_dp_ : \n" << dDp_dp_a - dDp_dp << std::endl;
}

//dDv_dp = [0, 0, 0]


TEST_F(ProcessorIMU_jacobians, dDv_dv)
{
    using namespace wolf;
    Eigen::Map<Eigen::Quaternions> Dq0(NULL), dq0(NULL);
    Eigen::Matrix3s dDv_dv, dDv_dv_a;

    dDv_dv_a = deltas_jac.jacobian_delta_.block(6,6,3,3);
    remapJacDeltas_quat0(deltas_jac, Dq0, dq0);
    //dDv_dvx = ( dR*(v + dvx) - dR*(v))/dvx
    for(int i=0;i<3;i++)
        dDv_dv.block<3,1>(0,i) = ( (Dq0 * deltas_jac.delta_noisy_vect_(i+6).tail(3)) - (Dq0 * deltas_jac.delta0_.tail(3)) )/delta_noise(i+6);

    ASSERT_TRUE((dDv_dv - dDv_dv_a).isMuchSmallerThan(1,0.000001)) << "dDv_dv : \n" << dDv_dv << "\n dDv_dv_a :\n" << dDv_dv_a <<
     "\ndDv_dv_a - dDv_dv_ : \n" << dDv_dv_a - dDv_dv << std::endl;
}

//dDo_dp = dDo_dv = [0, 0, 0]

TEST_F(ProcessorIMU_jacobians, dDo_do)
{
    using namespace wolf;
    Eigen::Map<Eigen::Quaternions> Dq0(NULL), dq0(NULL), Dq_noisy(NULL), dq_noisy(NULL);
    Eigen::Matrix3s dDo_do, dDo_do_a;

    //dDo_dox = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta) * (dr(theta)*exp(dthetax,0,0)) )/dthetax
    remapJacDeltas_quat0(deltas_jac, Dq0, dq0);
    dDo_do_a = deltas_jac.jacobian_delta_.block(3,3,3,3);

    for(int i=0;i<3;i++){
        remapJacDeltas_quat(deltas_jac, Dq_noisy, dq_noisy, i+3);
        dDo_do.block<3,1>(0,i) = R2v( (Dq0.matrix() * dq0.matrix()).transpose() * (Dq0.matrix() * dq_noisy.matrix()) )/Delta_noise(i+3);
    }

    ASSERT_TRUE((dDo_do - dDo_do_a).isMuchSmallerThan(1,0.000001)) << "dDo_do : \n" << dDo_do << "\n dDo_do_a :\n" << dDo_do_a <<
     "\ndDo_do_a - dDo_do_ : \n" << dDo_do_a - dDo_do << std::endl;
}


int main(int argc, char **argv)
{
    using namespace wolf;

     testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
}
