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

    //Eigen::Map<Eigen::Quaternions> q_in_1_, q_in_2_;

    Eigen::VectorXs delta_preint_plus_delta;
    Eigen::VectorXs delta_preint;
    Eigen::Matrix3s dDp_dab;
    Eigen::Matrix3s dDv_dab;
    Eigen::Matrix3s dDp_dwb;
    Eigen::Matrix3s dDv_dwb;
    Eigen::Matrix3s dDq_dwb;

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
        std::vector<bool> dDv_dab_check;
        std::vector<bool> dDp_dwb_check;
        std::vector<bool> dDv_dwb_check;

        dDp_dab_check.clear();
        dDv_dab_check.clear();
        dDp_dab_check.reserve(3);
        dDv_dab_check.reserve(3);

        dDp_dwb_check.clear();
        dDv_dwb_check.clear();
        dDp_dwb_check.reserve(3);
        dDv_dwb_check.reserve(3);

        for(int i = 0; i<3; i++){
            if((dDp_dab - bias_jac.dDp_dab_vect(i+1)) < wolf::Constants::EPS)
                dDp_dab_check.pushback(true);
            else
                dDp_dab_check.pushback(false);

            if((dDv_dab) - bias_jac.dDv_dab_vect(i+1) < wolf::Constants::EPS)
                dDv_dab_check.pushback(true);
            else
                dDv_dab_check.pushback(false);
            
            if((dDp_dwb - bias_jac.dDp_dwb_vect(i+3)) < wolf::Constants::EPS)
                dDp_dwb_check.pushback(true);
            else
                dDp_dwb_check.pushback(false);

            if((dDv_dwb - bias_jac.dDv_dwb_vect(i+3)) < wolf::Constants::EPS)
                dDv_dwb_check.pushback(true);
            else
                dDv_dwb_check.pushback(false);
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

    ///TODO : NOT WORKING - FIX ME
    /*new (&q_in_1_) Eigen::Map<Eigen::Quaternions>(bias_jac.delta_preint_vect(0).data() + 6);
    for(int i=0;i<3;i++){
        new (&q_in_2_) Eigen::Map<Eigen::Quaternions>(bias_jac.delta_preint_plus_delta_vect(i+1).data() + 6);
        dDq_dwb.block<3,1>(0,i) = R2v( q_in_1_.matrix().transpose() * q_in_2_.matrix())/ddelta_bias;
     }*/

     /*
        This jacobian must be checked :

        std::vector<bool> dDq_dwb_check;
        dDq_dwb_check.clear()
        dDq_dwb_check.reserve(6);

        for(int i = 0; i<6; i++){
            if((dDq_dwb - dDq_dwb_vect(i+1)) < wolf::Constants::EPS)
                dDq_dwb_check.pushback(true);
            else
                dDq_dwb_check.pushback(false);
        }
      */

    /*              Numerical method to check jacobians wrt noise

                                                            dDp_dP = [dDp_dPx, dDp_dPy, dDp_dPz]
    dDp_dPx = ((p + dPx) - p)/dPx = Id
    dDp_dPy = ((p + dPy) - p)/dPy = Id
    dDp_dPz = ((p + dPz) - p)/dPz = Id

                                                            dDp_dV = [dDp_dVx, dDp_dVy, dDp_dVz]
    dDp_dVx = ((v + dVx)*dt - v*dt)/dVx = Id*dt
    dDp_dVy = ((v + dVy)*dt - v*dt)/dVy = Id*dt
    dDp_dVz = ((v + dVz)*dt - v*dt)/dVz = Id*dt

                                                            dDp_dO = [dDp_dOx, dDp_dOy, dDp_dOz]
    dDp_dOx = (( dR(Theta + dthetax)*dp ) - ( dR(Theta)*dp ))/dthetax 
            = (( dR(Theta) * exp(dthetax,0,0)*dp ) - ( dR(Theta)*dp ))/dthetax
    dDp_dOy = (( dR(Theta) * exp(0,dthetay,0)*dp ) - ( dR(Theta)*dp ))/dthetay
    dDp_dOz = (( dR(Theta) * exp(0,0,dthetaz)*dp ) - ( dR(Theta)*dp ))/dthetaz

                                                            dDv_dP = [dDv_dPx, dDv_dPy, dDv_dPz] = [0, 0, 0]

                                                            dDv_dV = [dDv_dVx, dDv_dVy, dDv_dVz]
    dDv_dVx = ((v + dVx) - p)/dVx = Id
    dDv_dVy = ((v + dVy) - p)/dVy = Id
    dDv_dVz = ((v + dVz) - p)/dVz = Id
    
                                                            dDv_dO = [dDv_dOx, dDv_dOy, dDv_dOz]
    dDv_dOx = (( dR(Theta + dthetax)*dv ) - ( dR(Theta)*dv ))/dthetax 
            = (( dR(Theta) * exp(dthetax,0,0)*dv ) - ( dR(Theta)*dv ))/dthetax
    dDv_dOx = (( dR(Theta) * exp(0,dthetay,0)*dv ) - ( dR(Theta)*dv ))/dthetay
    dDv_dOz = (( dR(Theta) * exp(0,0,dthetaz)*dv ) - ( dR(Theta)*dv ))/dthetaz

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

            TODO : DOUBLE-CHECK THE FOLLOWING !!
                                                            dDo_dO = [dDo_dOx, dDo_dOy, dDo_dOz]
    dDo_dOx = log( dR(Theta).transpose() * dR(Theta+dThetax) )/dThetax
            = log( dR(Theta).transpose() * dR(Theta)*exp(dThetax,0,0) )/dThetax = Idx
    dDo_dOy = log( dR(Theta).transpose() * dR(Theta)*exp(0,dThetay,0) )/dThetay = Idy
    dDo_dOz = log( dR(Theta).transpose() * dR(Theta)*exp(0,0,dThetaz) )/dThetaz = Idz

                                                            dDo_do = [dDo_dox, dDo_doy, dDo_doz]
    dDo_dox = log( dR(Theta+dThetax).transpose() * dR(Theta) )/dThetax
            = log( (dR(Theta)*exp(dThetax,0,0)).transpose() * dR(Theta) )/dThetax
    dDo_doy = log( (dR(Theta)*exp(0,dThetay,0)).transpose() * dR(Theta) )/dThetay
    dDo_doz = log( (dR(Theta)*exp(0,0,dThetaz)).transpose() * dR(Theta) )/dThetaz
     */
    delete wolf_problem_ptr_;

    return 0;
}