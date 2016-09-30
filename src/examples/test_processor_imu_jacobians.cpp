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

using namespace wolf;

void remapJacDeltas_quat0(IMU_jac_deltas& _jac_delta, Eigen::Map<Eigen::Quaternions>& _Dq0, Eigen::Map<Eigen::Quaternions>& _dq0);
void remapJacDeltas_quat(IMU_jac_deltas& _jac_delta, Eigen::Map<Eigen::Quaternions>& _Dq, Eigen::Map<Eigen::Quaternions>& _dq, const int& place );

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
    //processor_imu.setOrigin(x0, t);
    wolf::Scalar ddelta_bias = 0.0001;
    wolf::Scalar dt = 0.001;
    struct IMU_jac_bias bias_jac = processor_imu.finite_diff_ab(dt, data_, ddelta_bias);

    Eigen::Map<Eigen::Quaternions> Dq0(NULL);
    Eigen::Map<Eigen::Quaternions> dq0(NULL);
    Eigen::Map<Eigen::Quaternions> Dq_noisy(NULL);
    Eigen::Map<Eigen::Quaternions> dq_noisy(NULL);
    Eigen::Map<Eigen::Quaternions> q_in_1(NULL), q_in_2(NULL);

    /* IMU_jac_deltas struct form :
    contains vectors of size 7 :
    Elements at place 0 are those not affected by the bias noise that we add (da_bx,..., dw_bx,... ).
                place 1 : added da_bx in data         place 2 : added da_by in data       place 3 : added da_bz in data
                place 4 : added dw_bx in data         place 5 : added dw_by in data       place 6 : added dw_bz in data
    */

    Eigen::Matrix3s dDp_dab, dDv_dab, dDp_dwb, dDv_dwb, dDq_dwb;

    /*
        dDp_dab = [dDp_dab_x, dDp_dab_y, dDp_dab_z]
        dDp_dab_x = (dDp(ab_x + dab_x, ab_y, ab_z) - dDp(ab_x,ab_y,ab_z)) / dab_x
        dDp_dab_x = (dDp(ab_x, ab_y + dab_y, ab_z) - dDp(ab_x,ab_y,ab_z)) / dab_y
        dDp_dab_x = (dDp(ab_x, ab_y, ab_z + dab_z) - dDp(ab_x,ab_y,ab_z)) / dab_z

        similar for dDv_dab
        note dDp_dab_x, dDp_dab_y, dDp_dab_z, dDv_dab_x, dDv_dab_y, dDv_dab_z are 3x1 vectors !

        dDq_dab = 0_{3x3}
        dDq_dwb = [dDq_dwb_x, dDq_dwb_y, dDq_dwb_z]
        dDq_dwb_x = log( dR(wb).trans() * dR(wb - dwb_x))/dwb_x
                  = log( dR(wb).trans() * exp((wx - wbx - dwb_x)dt, (wy - wby)dt, (wy - wby)dt))/dwb_x
        dDq_dwb_y = log( dR(wb).trans() * dR(wb - dwb_y))/dwb_y
        dDq_dwb_z = log( dR(wb).trans() * dR(wb + dwb_z))/dwb_z
     */


     new (&q_in_1) Eigen::Map<Eigen::Quaternions>(bias_jac.Delta0_.data() + 6);
     for(int i=0;i<3;i++){
         dDp_dab.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i).head(3) - bias_jac.Delta0_.head(3))/ddelta_bias;
         dDv_dab.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i).segment(3,3) - bias_jac.Delta0_.segment(3,3))/ddelta_bias;

         dDp_dwb.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i+3).head(3) - bias_jac.Delta0_.head(3))/ddelta_bias;
         dDv_dwb.block<3,1>(0,i) = (bias_jac.Deltas_noisy_vect_(i+3).segment(3,3) - bias_jac.Delta0_.segment(3,3))/ddelta_bias;

         new (&q_in_2) Eigen::Map<Eigen::Quaternions>(bias_jac.Deltas_noisy_vect_(i+3).data() + 6);
         dDq_dwb.block<3,1>(0,i) = R2v( q_in_1.matrix().transpose() * q_in_2.matrix())/ddelta_bias;
     }

     /*
        We just computed dDp_dab, dDp_dwb, dDv_dab and dDv_dwb
        These must be compared to elements in IMU_jac_bias struct

        bool dDp_dab_check;
        bool dDv_dab_check;
        bool dDp_dwb_check;
        bool dDv_dwb_check;
        bool dDq_dwb_check;

        if((dDp_dab - bias_jac.dDp_dab_) < wolf::Constants::EPS)
            dDp_dab_check = true;
        else
            dDp_dab_check = false;

        if((dDv_dab - bias_jac.dDv_dab_) < wolf::Constants::EPS)
                dDv_dab_check = true;
            else
                dDv_dab_check = false;

        if((dDp_dwb - bias_jac.dDp_dwb_) < wolf::Constants::EPS)
            dDp_dwb_check = true;
        else
            dDp_dwb_check = false;

        if((dDv_dwb - bias_jac.dDv_dwb_) < wolf::Constants::EPS)
            dDv_dwb_check = true;
        else
            dDv_dwb_check = false;

        if((dDq_dwb - bias_jac.dDq_dwb_) < wolf::Constants::EPS)
            dDq_dwb_check = true;
        else
            dDq_dwb_check = false;
      */

    if(dDp_dab.isApprox(bias_jac.dDp_dab_, wolf::Constants::EPS) )
        std::cout<< "dDp_dab_ jacobian is correct !" << std::endl;
    else{
        std::cout<< "dDp_dab_ jacobian is not correct ..." << std::endl;
        std::cout << "dDp_dab : \n" << dDp_dab << "\n bias_jac.dDp_dab_ :\n" << bias_jac.dDp_dab_ << std::endl;
    }

    if(dDv_dab.isApprox(bias_jac.dDv_dab_, wolf::Constants::EPS) )
        std::cout<< "dDv_dab_ jacobian is correct !" << std::endl;
    else{
        std::cout<< "dDv_dab_ jacobian is not correct ..." << std::endl;
        std::cout << "dDv_dab_ : \n" << dDv_dab << "\n bias_jac.dDv_dab_ :\n" << bias_jac.dDv_dab_ << std::endl;
    }

    if(dDp_dwb.isApprox(bias_jac.dDp_dwb_, wolf::Constants::EPS) )
        std::cout<< "dDp_dwb_ jacobian is correct !" << std::endl;
    else{
        std::cout<< "dDp_dwb_ jacobian is not correct ..." << std::endl;
        std::cout << "dDp_dwb_ : \n" << dDp_dwb << "\n bias_jac.dDp_dwb_ :\n" << bias_jac.dDp_dwb_ << std::endl;
    }

    if(dDv_dwb.isApprox(bias_jac.dDv_dwb_, wolf::Constants::EPS) )
        std::cout<< "dDv_dwb_ jacobian is correct !" << std::endl;
    else{
        std::cout<< "dDv_dwb_ jacobian is not correct ..." << std::endl;
        std::cout << "dDv_dwb_ : \n" << dDv_dwb << "\n bias_jac.dDv_dwb_ :\n" << bias_jac.dDv_dwb_ << std::endl;
    }

    if(dDq_dwb.isApprox(bias_jac.dDq_dwb_, wolf::Constants::EPS) )
        std::cout<< "dDq_dwb_ jacobian is correct !" << std::endl;
    else{
        std::cout<< "dDq_dwb_ jacobian is not correct ..." << std::endl;
        std::cout << "dDq_dwb_ : \n" << dDq_dwb << "\n bias_jac.dDq_dwb_ :\n" << bias_jac.dDq_dwb_ << std::endl;
    }


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

            TODO : DOUBLE-CHECK THE FOLLOWING !!
                                                            dDo_dO = [dDo_dOx, dDo_dOy, dDo_dOz]
    dDo_dOx = log( dR(Theta).transpose() * dR(Theta+dThetax) )/dThetax
            = log( dR(Theta).transpose() * dR(Theta)*exp(dThetax,0,0) )/dThetax = Idx
    dDo_dOy = log( dR(Theta).transpose() * dR(Theta)*exp(0,dThetay,0) )/dThetay = Idy
    dDo_dOz = log( dR(Theta).transpose() * dR(Theta)*exp(0,0,dThetaz) )/dThetaz = Idz
                                                            
                                                            //other solution to investigate
    dDo_dOx = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta+dThetax) * dr(theta) )/dThetax
            = log( (dR(Theta) * dr(theta)).transpose() * (dR(Theta)*exp(dThetax,0,0)) * dr(theta) )/dThetax
            = log( (_Delta * _delta).transpose() * (_Delta_noisy * _delta))
    dDo_dOy = log( (dR(Theta) * dr(theta)).transpose() * (dR(Theta)*exp(0,dThetay,0)) * dr(theta) )/dThetay
    dDo_dOz = log( (dR(Theta) * dr(theta)).transpose() * (dR(Theta)*exp(0,0,dThetaz)) * dr(theta) )/dThetaz

                                                            dDo_do = [dDo_dox, dDo_doy, dDo_doz]
    dDo_dox = log( dR(Theta+dThetax).transpose() * dR(Theta) )/dThetax
            = log( (dR(Theta)*exp(dThetax,0,0)).transpose() * dR(Theta) )/dThetax
    dDo_doy = log( (dR(Theta)*exp(0,dThetay,0)).transpose() * dR(Theta) )/dThetay
    dDo_doz = log( (dR(Theta)*exp(0,0,dThetaz)).transpose() * dR(Theta) )/dThetaz

                                                            //other solution to investigate
    dDo_dox = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta) * dr(theta+dthetax) )/dthetax
            = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta) * (dr(theta)*exp(dthetax,0,0)) )/dthetax
            = log( (_Delta * _delta).transpose() * (_Delta * _delta_noisy))
    dDo_doy = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta) * (dr(theta)*exp(0,dthetay,0)) )/dthetay
    dDo_doz = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta) * (dr(theta)*exp(0,0,dthetaz)) )/dthetaz

     */

     //taking care of noise now 
    Eigen::Matrix<wolf::Scalar,9,1> Delta_noise;
    Eigen::Matrix<wolf::Scalar,9,1> delta_noise;
    Delta_noise << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.002, 0.002, 0.002;
    delta_noise << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.002, 0.002, 0.002;

    struct IMU_jac_deltas deltas_jac = processor_imu.finite_diff_noise(dt, data_, Delta_noise, delta_noise);

    /* reminder : 
                            jacobian_delta_preint_vect_                                                            jacobian_delta_vect_
                            0: + 0,                                                                                 0: + 0
                            1: +dPx, 2: +dPy, 3: +dPz                                                               1: + dpx, 2: +dpy, 3: +dpz
                            4: +dVx, 5: +dVy, 6: +dVz                                                               4: + dvx, 5: +dvy, 6: +dvz
                            7: +dOx, 8: +dOy, 9: +dOz                                                               7: + dox, 8: +doy, 9: +doz
    */

    Eigen::Matrix3s dDp_dP, dDp_dV, dDp_dO, dDv_dP, dDv_dV, dDv_dO, dDo_dP, dDo_dV, dDo_dO;
    Eigen::Matrix3s dDp_dp, dDp_dv, dDp_do, dDv_dp, dDv_dv, dDv_do, dDo_dp, dDo_dv, dDo_do; 

    remapJacDeltas_quat0(deltas_jac, Dq0, dq0);

    //dDp_dP and dDv_dV
    for(int i = 0; i < 3; i++){

        //dDp_dPx = ((P + dPx) - P)/dPx
        dDp_dP.block<3,1>(0,i) = (deltas_jac.Delta_noisy_vect_(i).head(3) - deltas_jac.Delta0_.head(3))/Delta_noise(i);
        //Dp_dVx = ((V + dVx)*dt - V*dt)/dVx
        dDp_dV.block<3,1>(0,i) = (deltas_jac.Delta_noisy_vect_(i+3).segment(3,3)*dt - deltas_jac.Delta0_.segment(3,3)*dt)/Delta_noise(i+3);
        //dDp_dOx = (( dR(Theta) * exp(dThetax,0,0)*dp ) - ( dR(Theta)*dp ))/dThetax
        remapJacDeltas_quat(deltas_jac, Dq_noisy, dq_noisy, i+6);
        dDp_dO.block<3,1>(0,i) = (Dq_noisy.matrix() * deltas_jac.delta0_.head(3)) - (Dq0.matrix()* deltas_jac.delta0_.head(3))/Delta_noise(i+6);

        //dDv_dP = [0, 0, 0]
        //dDv_dVx = ((V + dVx) - V)/dVx
        dDv_dV.block<3,1>(0,i) = (deltas_jac.Delta_noisy_vect_(i+3).segment(3,3) - deltas_jac.Delta0_.segment(3,3))/Delta_noise(i+3);
        //dDv_dOx = (( dR(Theta) * exp(dThetax,0,0)*dv ) - ( dR(Theta)*dv ))/dThetax
        dDv_dO.block<3,1>(0,i) = (Dq_noisy.matrix() * deltas_jac.delta0_.segment(3,3)) - (Dq0.matrix()* deltas_jac.delta0_.segment(3,3))/Delta_noise(i+6);

        //dDo_dP = dDo_dV = [0, 0, 0]
        //dDo_dOx = log( (dR(Theta) * dr(theta)).transpose() * (dR(Theta)*exp(dThetax,0,0)) * dr(theta) )/dThetax
        dDo_dO.block<3,1>(0,i) = R2v( (Dq0.matrix() * dq0.matrix()).transpose() * (Dq_noisy.matrix() * dq0.matrix()) )/Delta_noise(i+6);

        //dDp_dpx = ( dR*(p + dpx) - dR*(p))/dpx
        dDp_dp.block<3,1>(0,i) = ( (Dq0.matrix() * deltas_jac.delta_noisy_vect_(i).head(3)) - (Dq0.matrix() * deltas_jac.delta0_.head(3)) )/delta_noise(i);
        //dDp_dv = dDp_do = [0, 0, 0]

        //dDv_dp = [0, 0, 0]
        //dDv_dvx = ( dR*(v + dvx) - dR*(v))/dvx
        dDv_dv.block<3,1>(0,i) = ( (Dq0 * deltas_jac.delta_noisy_vect_(i+3).segment(3,3)) - (Dq0 * deltas_jac.delta0_.segment(3,3)) )/delta_noise(i+3);
        //dDv_do = [0, 0, 0]

        //dDo_dp = dDo_dv = [0, 0, 0]
        //dDo_dox = log( (dR(Theta) * dr(theta)).transpose() * dR(Theta) * (dr(theta)*exp(dthetax,0,0)) )/dthetax
        dDo_do.block<3,1>(0,i) = R2v( (Dq0.matrix() * dq0.matrix()).transpose() * (Dq0.matrix() * dq_noisy.matrix()) )/Delta_noise(i+6);
    }

    /* jacobians wrt deltas have PVQ form :
    dDp_dP = deltas_jac.jacobian_delta_preint_.block(0,0,3,3);    dDp_dV = deltas_jac.jacobian_delta_preint_.block(0,3,3,3);        dDp_dO = deltas_jac.jacobian_delta_preint_.block(0,6,3,3);
    dDv_dP = deltas_jac.jacobian_delta_preint_.block(3,0,3,3);    dDv_dV = deltas_jac.jacobian_delta_preint_.block(3,3,3,3);        dDv_dO = deltas_jac.jacobian_delta_preint_.block(3,6,3,3);
    dDo_dP = deltas_jac.jacobian_delta_preint_.block(6,0,3,3);    dDo_dV = deltas_jac.jacobian_delta_preint_.block(6,3,3,3);        dDo_dO = deltas_jac.jacobian_delta_preint_.block(6,6,3,3);
     */

    if(dDp_dP.isApprox(deltas_jac.jacobian_delta_preint_.block(0,0,3,3), wolf::Constants::EPS) )
        std::cout<< "dDp_dP jacobian is correct !" << std::endl;
    else{
        std::cout<< "dDp_dP jacobian is not correct ..." << std::endl;
        std::cout << "dDp_dP : \n" << dDp_dP << "\n deltas_jac.jacobian_delta_preint_.block(0,0,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(0,0,3,3) << std::endl;
    }

    if(dDp_dV.isApprox(deltas_jac.jacobian_delta_preint_.block(0,3,3,3), wolf::Constants::EPS) )
        std::cout<< "dDp_dV jacobian is correct !" << std::endl;
    else{
        std::cout<< "dDp_dV jacobian is not correct ..." << std::endl;
        std::cout << "dDp_dV : \n" << dDp_dV << "\n deltas_jac.jacobian_delta_preint_.block(0,3,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(0,3,3,3) << std::endl;
    }

    if(dDp_dO.isApprox(deltas_jac.jacobian_delta_preint_.block(0,6,3,3), wolf::Constants::EPS) )
        std::cout<< "dDp_dO jacobian is correct !" << std::endl;
    else{
        std::cout<< "dDp_dO jacobian is not correct ..." << std::endl;
        std::cout << "dDp_dO : \n" << dDp_dO << "\n deltas_jac.jacobian_delta_preint_.block(0,6,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(0,6,3,3) << std::endl;
    }

    if(dDv_dV.isApprox(deltas_jac.jacobian_delta_preint_.block(3,3,3,3), wolf::Constants::EPS) )
        std::cout<< "dDv_dV jacobian is correct !" << std::endl;
    else{
        std::cout<< "dDv_dV jacobian is not correct ..." << std::endl;
        std::cout << "dDv_dV : \n" << dDv_dV << "\n deltas_jac.jacobian_delta_preint_.block(3,3,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(3,3,3,3) << std::endl;
    }

    if(dDv_dO.isApprox(deltas_jac.jacobian_delta_preint_.block(3,6,3,3), wolf::Constants::EPS) )
        std::cout<< "dDv_dO jacobian is correct !" << std::endl;
    else{
        std::cout<< "dDv_dO jacobian is not correct ..." << std::endl;
        std::cout << "dDv_dO : \n" << dDv_dO << "\n deltas_jac.jacobian_delta_preint_.block(3,6,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(3,6,3,3) << std::endl;
    }

    if(dDo_dO.isApprox(deltas_jac.jacobian_delta_preint_.block(6,6,3,3), wolf::Constants::EPS) )
        std::cout<< "dDo_dO jacobian is correct !" << std::endl;
    else{
        std::cout<< "dDo_dO jacobian is not correct ..." << std::endl;
        std::cout << "dDo_dO : \n" << dDo_dO << "\n deltas_jac.jacobian_delta_preint_.block(6,6,3,3) :\n" << deltas_jac.jacobian_delta_preint_.block(6,6,3,3) << std::endl;
    }

    delete wolf_problem_ptr_;

    return 0;
}

using namespace wolf;

void remapJacDeltas_quat0(IMU_jac_deltas& _jac_delta, Eigen::Map<Eigen::Quaternions>& _Dq0, Eigen::Map<Eigen::Quaternions>& _dq0){

        new (&_Dq0) Eigen::Map<const Eigen::Quaternions>(_jac_delta.Delta0_.data() + 6);
        new (&_dq0) Eigen::Map<const Eigen::Quaternions>(_jac_delta.delta0_.data() + 6);
}

void remapJacDeltas_quat(IMU_jac_deltas& _jac_delta, Eigen::Map<Eigen::Quaternions>& _Dq, Eigen::Map<Eigen::Quaternions>& _dq, const int& place ){
    
    assert(place < _jac_delta.Delta_noisy_vect_.size());
    new (&_Dq) Eigen::Map<const Eigen::Quaternions>(_jac_delta.Delta_noisy_vect_(place).data() + 6);
    new (&_dq) Eigen::Map<const Eigen::Quaternions>(_jac_delta.delta_noisy_vect_(place).data() + 6);
}