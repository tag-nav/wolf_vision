/**
 * \file test_processor_imu_jacobians.cpp
 *
 *  Created on: Sep 26, 2016
 *      \author: AtDinesh
 */

//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"

//std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>

//#define DEBUG_RESULTS

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << std::endl << "==================== processor IMU : Checking Jacobians ======================" << std::endl;

    TimeStamp t;
    Eigen::Vector6s data_;
    data_ << 0,0,0, 0,0,0;

    // Wolf problem
    Problem* wolf_problem_ptr_ = new Problem(FRM_PVQBB_3D);
    Eigen::VectorXs IMU_extrinsics(7);
    IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
    //SensorBase* sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, nullptr);
    //wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    // Set the origin
    t.set(0.0001); // clock in 0,1 ms ticks
    Eigen::VectorXs x0(16);
    x0 << 0,1,0,  1,0,0,  0,0,0,1,  0,0,.000,  0,0,.000;

    //wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);

    //CaptureIMU* imu_ptr;

    ProcessorIMU processor_imu;
    processor_imu.setOrigin(x0, t);
    wolf::Scalar ddelta_bias = 0.0001;
    struct IMU_jac_deltas bias_jac = processor_imu.finite_diff_ab(0.001, data_, ddelta_bias);

    /* IMU_jac_deltas struct form :
    contains vectors of size 7 :
    Elements at place 0 are those not affected by the bias noise that we add (da_bx,..., dw_bx,... ).
              place 1 : added da_bx in data
              place 2 : added da_by in data
              place 3 : added da_bz in data
              place 4 : added dw_bx in data
              place 5 : added dw_by in data
              place 6 : added dw_bz in data
    */

    Eigen::VectorXs delta_preint_plus_delta;
    Eigen::VectorXs delta_preint;
    Eigen::Matrix3s dDp_dab;
    Eigen::Matrix3s dDv_dab;
    Eigen::Matrix3s dDp_dwb;
    Eigen::Matrix3s dDv_dwb;

    /*
        dDp_dab = [dDp_dab_x, dDp_dab_y, dDp_dab_z]
        dDp_dab_x = (dDp(ab_x + dab_x, ab_y, ab_z) - dDp(ab_x,ab_y,ab_z)) / dab_x
        dDp_dab_x = (dDp(ab_x, ab_y + dab_y, ab_z) - dDp(ab_x,ab_y,ab_z)) / dab_y
        dDp_dab_x = (dDp(ab_x, ab_y, ab_z + dab_z) - dDp(ab_x,ab_y,ab_z)) / dab_z

        similar for dDv_dab
        note dDp_dab_x, dDp_dab_y, dDp_dab_z, dDv_dab_x, dDv_dab_y, dDv_dab_z are 3x1 vectors !
     */

     for(int i=0;i<3;i++){
         dDp_dab.block<3,1>(0,i) = (bias_jac.delta_preint_plus_delta_vect(i+1).head(3) - bias_jac.delta_preint_vect(0).head(3))/ddelta_bias;
         dDv_dab.block<3,1>(0,i) = (bias_jac.delta_preint_plus_delta_vect(i+1).segment(3,3) - bias_jac.delta_preint_vect(0).segment(3,3))/ddelta_bias;

         dDp_dwb.block<3,1>(0,i) = (bias_jac.delta_preint_plus_delta_vect(i+3).head(3) - bias_jac.delta_preint_vect(0).head(3))/ddelta_bias;
         dDv_dwb.block<3,1>(0,i) = (bias_jac.delta_preint_plus_delta_vect(i+3).segment(3,3) - bias_jac.delta_preint_vect(0).segment(3,3))/ddelta_bias;
     }

     /*
        We just computed dDp_dab, dDp_dwb, dDv_dab and dDv_dwb
        These must be compared to elements in dDp_dab_vect, dDv_dab_vect, dDp_dwb_vect and dDv_dwb_vect;

        std::vector<bool> dDp_dab_check;
        dDp_dab_check.clear()
        dDp_dab_check.reserve(6);

        for(int i = 0; i<6; i++){
            if((dDp_dab - dDp_dab_vect(i+1)) < wolf::Constants::EPS)
                dDp_dab_check.pushback(true);
            else
                dDp_dab_check.pushback(false);
        }
      */

      /*
        dDq_dab = 0_{3x3}
        dDq_dwb = [dDq_dwb_x, dDq_dwb_y, dDq_dwb_z]
        dDq_dwb_x = log( dR(wb).trans() * dR(wb - dwb_x))/dwb_x
                  = log( dR(wb).trans() * exp((wx - wbx - dwb_x)dt, (wy - wby)dt, (wy - wby)dt))/dwb_x
        dDq_dwb_y = log( dR(wb).trans() * dR(wb - dwb_y))/dwb_y
        dDq_dwb_z = log( dR(wb).trans() * dR(wb + dwb_z))/dwb_z
       */

    delete wolf_problem_ptr_;

    return 0;
}