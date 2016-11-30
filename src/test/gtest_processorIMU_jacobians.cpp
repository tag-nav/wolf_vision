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
class ProcessorIMU_jacobians_bias : public testing::Test
{
    public:
        TimeStamp t;
        Eigen::Vector6s data_;
        struct IMU_jac_bias bias_jac;
        wolf::Scalar ddelta_bias;
        wolf::Scalar dt;

        void remapJacDeltas_quat0(IMU_jac_deltas& _jac_delta, Eigen::Map<Eigen::Quaternions>& _Dq0, Eigen::Map<Eigen::Quaternions>& _dq0){

            new (&_Dq0) Eigen::Map<const Eigen::Quaternions>(_jac_delta.Delta0_.data() + 6);
            new (&_dq0) Eigen::Map<const Eigen::Quaternions>(_jac_delta.delta0_.data() + 6);
        }

        void remapJacDeltas_quat(IMU_jac_deltas& _jac_delta, Eigen::Map<Eigen::Quaternions>& _Dq, Eigen::Map<Eigen::Quaternions>& _dq, const int& place ){
    
            assert(place < _jac_delta.Delta_noisy_vect_.size());
            new (&_Dq) Eigen::Map<const Eigen::Quaternions>(_jac_delta.Delta_noisy_vect_(place).data() + 6);
            new (&_dq) Eigen::Map<const Eigen::Quaternions>(_jac_delta.delta_noisy_vect_(place).data() + 6);
        }

    virtual void SetUp()
    {
        //SetUp for jacobians wrt bias testing
        wolf::Scalar deg_to_rad = M_PI/180.0;
        data_ << 10,0.5,3, 100*deg_to_rad,110*deg_to_rad,30*deg_to_rad;

        // Wolf problem
        ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs IMU_extrinsics(7);
        IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot

        ProcessorIMU_UnitTester processor_imu;
        ddelta_bias = 0.00000001;
        dt = 0.001;

        //defining a random Delta to begin with (not to use Origin point)
        Eigen::Matrix<wolf::Scalar,10,1> Delta0;
        Delta0 = Eigen::Matrix<wolf::Scalar,10,1>::Random();
        Delta0.head<3>() = Delta0.head<3>()*100;
        Delta0.segment<3>(3) = Delta0.segment<3>(3)*10;
        Eigen::Vector3s ang0, ang;
        ang0 << 120.08*deg_to_rad, 12.36*deg_to_rad, 54.32*deg_to_rad; 

        Eigen::Map<Eigen::Quaternions> Delta0_quat(Delta0.data()+6);
        Delta0_quat = v2q(ang0);
        Delta0_quat.normalize();
        ang = q2v(Delta0_quat);

        std::cout << "\ninput Delta0 : " << Delta0 << std::endl;
        std::cout << "\n rotation vector we start with :\n" << ang << std::endl;

        struct IMU_jac_bias bias_jac_c = processor_imu.finite_diff_ab(dt, data_, ddelta_bias, Delta0);
        bias_jac.copyfrom(bias_jac_c);
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


// A new one of these is created for each test
class ProcessorIMU_jacobians_noise : public testing::Test
{
    public:
        TimeStamp t;
        Eigen::Vector6s data_;         
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

TEST_F(ProcessorIMU_jacobians_bias, dDp_dab)
{
    using namespace wolf;
    Eigen::Matrix3s dDp_dab;

    for(int i=0;i<3;i++)
         dDp_dab.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i).head(3) - bias_jac.Delta0_.head(3))/ddelta_bias;

    EXPECT_TRUE((dDp_dab - bias_jac.dDp_dab_).isMuchSmallerThan(1,0.000001)) << "dDp_dab : \n" << dDp_dab << "\n bias_jac.dDp_dab_ :\n" << bias_jac.dDp_dab_ <<
     "\ndDp_dab_a - dDp_dab_ : \n" << bias_jac.dDp_dab_ - dDp_dab << std::endl;
}

TEST_F(ProcessorIMU_jacobians_bias, dDv_dab)
{
    using namespace wolf;
    Eigen::Matrix3s dDv_dab;

    for(int i=0;i<3;i++)
         dDv_dab.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i).tail(3) - bias_jac.Delta0_.tail(3))/ddelta_bias;

    EXPECT_TRUE((dDv_dab - bias_jac.dDv_dab_).isMuchSmallerThan(1,0.000001)) << "dDv_dab : \n" << dDv_dab << "\n bias_jac.dDv_dab_ :\n" << bias_jac.dDv_dab_ <<
     "\ndDv_dab_a - dDv_dab_ : \n" << bias_jac.dDv_dab_ - dDv_dab << std::endl;
}

TEST_F(ProcessorIMU_jacobians_bias, dDp_dwb)
{
    using namespace wolf;
    Eigen::Matrix3s dDp_dwb;

    for(int i=0;i<3;i++)
         dDp_dwb.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i+3).head(3) - bias_jac.Delta0_.head(3))/ddelta_bias;

    EXPECT_TRUE((dDp_dwb - bias_jac.dDp_dwb_).isMuchSmallerThan(1,0.000001)) << "dDp_dwb : \n" << dDp_dwb << "\n bias_jac.dDp_dwb_ :\n" << bias_jac.dDp_dwb_ <<
     "\ndDp_dwb_a - dDv_dab_ : \n" << bias_jac.dDp_dwb_ - dDp_dwb << std::endl;
}

TEST_F(ProcessorIMU_jacobians_bias, dDv_dwb_)
{
    using namespace wolf;
    Eigen::Matrix3s dDv_dwb;

    for(int i=0;i<3;i++)
         dDv_dwb.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i+3).tail(3) - bias_jac.Delta0_.tail(3))/ddelta_bias;

    EXPECT_TRUE((dDv_dwb - bias_jac.dDv_dwb_).isMuchSmallerThan(1,0.000001)) << "dDv_dwb : \n" << dDv_dwb << "\n bias_jac.dDv_dwb_ :\n" << bias_jac.dDv_dwb_ <<
     "\ndDv_dwb_a - dDv_dwb_ : \n" << bias_jac.dDv_dwb_ - dDv_dwb << std::endl;
}

TEST_F(ProcessorIMU_jacobians_bias, dDq_dab)
{
    using namespace wolf;
    Eigen::Map<Eigen::Quaternions> q_in_1(NULL), q_in_2(NULL);
    Eigen::Matrix3s dDq_dab;

    new (&q_in_1) Eigen::Map<Eigen::Quaternions>(bias_jac.Delta0_.data() + 3);
    for(int i=0;i<3;i++){
        new (&q_in_2) Eigen::Map<Eigen::Quaternions>(bias_jac.Deltas_noisy_vect_(i).data() + 3);
        dDq_dab.block<3,1>(0,i) = R2v( q_in_1.matrix().transpose() * q_in_2.matrix())/ddelta_bias;
    }

    EXPECT_TRUE(dDq_dab.isZero(wolf::Constants::EPS)) << "\t\tdDq_dab_ jacobian is not Zero :" << dDq_dab << std::endl;
}

TEST_F(ProcessorIMU_jacobians_bias, dDq_dwb)
{
    using namespace wolf;
    Eigen::Map<Eigen::Quaternions> q_in_1(NULL), q_in_2(NULL);
    Eigen::Matrix3s dDq_dwb;

    new (&q_in_1) Eigen::Map<Eigen::Quaternions>(bias_jac.Delta0_.data() + 3);
    for(int i=0;i<3;i++){
        new (&q_in_2) Eigen::Map<Eigen::Quaternions>(bias_jac.Deltas_noisy_vect_(i+3).data() + 3);
        dDq_dwb.block<3,1>(0,i) = R2v( q_in_1.matrix().transpose() * q_in_2.matrix())/ddelta_bias;
    }

    EXPECT_TRUE((dDq_dwb - bias_jac.dDq_dwb_).isMuchSmallerThan(1,0.000001)) << "dDq_dwb : \n" << dDq_dwb << "\n bias_jac.dDq_dwb_ :\n" << bias_jac.dDq_dwb_ <<
     "\ndDq_dwb_a - dDq_dwb_ : \n" << bias_jac.dDq_dwb_ - dDq_dwb << std::endl;
}


int main(int argc, char **argv)
{
    using namespace wolf;

     testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
}