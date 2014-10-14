/*
 * \file test_error_states.cpp
 *
 *  Created on: 03/06/2014
 *      \author: jsola
 */


#include <iostream>
#include <eigen3/Eigen/Dense>

#include "state_error_pqv.h"
#include "state_error_imu.h"

using namespace std;
using namespace Eigen;

int main(){
    // We show how to manipulate error states
    cout << "\nError states - demo";
    cout << "\n-------------------\n" << endl;
    cout << "check in-class static nominal state size for PQV: " << StateErrorPQV::SIZE_NOMINAL_ << endl;
    cout << "check in-class static error state size for PQV: " << StateErrorPQV::SIZE_ERROR_ << endl;
    cout << "check in-class static nominal state size for IMU: " << StateErrorIMU::SIZE_NOMINAL_ << endl;
    cout << "check in-class static error state size for IMU: " << StateErrorIMU::SIZE_ERROR_ << endl;

    cout << "\n---Checking remote states---" << endl;
    VectorXs storage(20);
    storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
    cout << "storage   : " << storage.transpose() << endl;
    unsigned int index = 0;
    VectorXs pqvnomstate(StateErrorPQV::SIZE_NOMINAL_);
    pqvnomstate << 20, 21, 22, 23, 24, 25, 26, 27, 28, 29;
    VectorXs imunomstate(StateErrorIMU::SIZE_NOMINAL_);
    imunomstate << 50, 51, 52, 53, 54, 55;
    StateErrorPQV pqv(storage, index, pqvnomstate);
    pqv.qn().normalize();
    pqv.clearError();
    StateErrorIMU imu(storage, index + StateErrorPQV::SIZE_ERROR_, imunomstate);
    cout << "storage   : " << storage.transpose() << endl;
    imu.clearError();
    cout << "storage   : " << storage.transpose() << endl;
    cout << "pqv  state: " << pqv.xc().transpose() << endl;
    cout << "pqv  pos  : " << pqv.pc().transpose() << endl;
    cout << "pqv  vel  : " << pqv.vc().transpose() << endl;
    cout << "pqv  quat : " << pqv.qc().coeffs().transpose() << endl;
    cout << "imu  state: " << imu.xc().transpose() << endl;
    cout << "imu  abias: " << imu.abc().transpose() << endl;
    cout << "imu  wbias: " << imu.wbc().transpose() << endl;

    pqv.xe() << 1,1,1,1,1,1,1,1,1;
    pqv.xe() *= 0.1;
    cout << "pqv xn: " << pqv.xn().transpose() << endl;
    cout << "pqv xe: " << pqv.xe().transpose() << endl;
    cout << "pqv xc: " << pqv.xc().transpose() << endl;
    cout << "pqv qn norm: " << pqv.qn().norm() << endl;
    cout << "pqv qc norm: " << pqv.qc().norm() << endl;

    cout << "doing 'storage *= 2' ..." << endl;
    storage *= 2;
    cout << "pqv xn: " << pqv.xn().transpose() << endl;
    cout << "pqv xe: " << pqv.xe().transpose() << endl;
    cout << "pqv xc: " << pqv.xc().transpose() << endl;
    cout << "pqv qn norm: " << pqv.qn().norm() << endl;
    cout << "pqv qc norm: " << pqv.qc().norm() << endl;


    cout << "\n---Checking local states---" << endl;
    StateErrorPQV pqvlocal(pqvnomstate);
    pqvlocal.qn().normalize();
    pqvlocal.clearError();
    StateErrorIMU imulocal(imunomstate);
    imulocal.clearError();
    cout << "storage   : " << storage.transpose() << endl;
    cout << "pqvlocal  state: " << pqvlocal.xc().transpose() << endl;
    cout << "pqvlocal  pos  : " << pqvlocal.pc().transpose() << endl;
    cout << "pqvlocal  vel  : " << pqvlocal.vc().transpose() << endl;
    cout << "pqvlocal  quat : " << pqvlocal.qc().coeffs().transpose() << endl;
    cout << "imulocal  state: " << imulocal.xc().transpose() << endl;
    cout << "imulocal  abias: " << imulocal.abc().transpose() << endl;
    cout << "imulocal  wbias: " << imulocal.wbc().transpose() << endl;

    pqvlocal.xe() << 1,1,1,1,1,1,1,1,1;
    pqvlocal.xe() *= 0.1;
    cout << "pqvlocal xn: " << pqvlocal.xn().transpose() << endl;
    cout << "pqvlocal xe: " << pqvlocal.xe().transpose() << endl;
    cout << "pqvlocal xc: " << pqvlocal.xc().transpose() << endl;
    cout << "pqvlocal qn norm: " << pqvlocal.qn().norm() << endl;
    cout << "pqvlocal qc norm: " << pqvlocal.qc().norm() << endl;

    cout << "doing 'pqvlocal.xe() *= 2' ..." << endl;
    pqvlocal.xe() *= 2;
    cout << "pqvlocal xn: " << pqvlocal.xn().transpose() << endl;
    cout << "pqvlocal xe: " << pqvlocal.xe().transpose() << endl;
    cout << "pqvlocal xc: " << pqvlocal.xc().transpose() << endl;
    cout << "pqvlocal qn norm: " << pqvlocal.qn().norm() << endl;
    cout << "pqvlocal qc norm: " << pqvlocal.qc().norm() << endl;

    return 0;
}
