#include <iostream>
#include "state_pqv.h"
#include "state_imu.h"
#include "state_error_pqv.h"

using namespace std;
using namespace Eigen;

int main()
{
    cout << "\nStates demo";
    cout << "\n-----------\n" << endl;

    cout << "check in-class static state size for PQV: " << StatePQV::SIZE_ << endl;
    cout << "check in-class static state size for IMU: " << StateIMU::SIZE_ << endl;


    cout << "---Checking remote states---" << endl;
    VectorXs storage(20);

    //  cout << "Check StateBase constructor with wrong args: " << endl;
    //  VectorXs smaller(5), equal(storage.size()), larger(storage.size()+5);
    //  StateBase basetest(&storage, 0, equal);
    //  StateBase basetest(&storage, 5, equal);
    //  StateBase basetest(&storage, 25, smaller);
    //  StateBase basetest(&storage, 0, larger);
    //  StateBase basetest(&storage, 17, smaller);
    //  StateBase basetest(&storage, 25, larger);

    storage << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
    unsigned int index = 0, bsize = 4;
    VectorXs pqvstate(StatePQV::SIZE_);
    pqvstate << 20, 21, 22, 23, 24, 25, 26, 27, 28, 29;
    VectorXs imustate(StateIMU::SIZE_);
    imustate << 50, 51, 52, 53, 54, 55;
    StatePQV pqv(storage, index + bsize, pqvstate);
    StateIMU imu(storage, index + bsize + StatePQV::SIZE_, imustate);
    cout << "storage   : " << storage.transpose() << endl;
    cout << "pqv  state: " << pqv.x().transpose() << endl;
    cout << "pqv  pos  : " << pqv.p().transpose() << endl;
    cout << "pqv  vel  : " << pqv.v().transpose() << endl;
    cout << "pqv  quat : " << pqv.q().coeffs().transpose() << endl;
    cout << "imu  state: " << imu.x().transpose() << endl;
    cout << "imu  abias: " << imu.ab().transpose() << endl;
    cout << "imu  wbias: " << imu.wb().transpose() << endl;
    cout << "doing 'base.x() *= 2'" << endl;

    cout << "doing 'pqv.x() *= 3'" << endl;
    pqv.x() *= 3;
    cout << "pqv  state: " << pqv.x().transpose() << endl;
    cout << "pqv  pos  : " << pqv.p().transpose() << endl;
    cout << "pqv  vel  : " << pqv.v().transpose() << endl;
    cout << "pqv  quat : " << pqv.q().coeffs().transpose() << endl;
    cout << "doing 'pqv.p().array() += 2' ; 'pqv.v() *= 2' ; 'pqv.q().normalize()'" << endl;
    pqv.p().array() += 2;
    pqv.v() *= 2;
    pqv.q().normalize();
    cout << "pqv  pos  : " << pqv.p().transpose() << endl;
    cout << "pqv  vel  : " << pqv.v().transpose() << endl;
    cout << "pqv  quat : " << pqv.q().coeffs().transpose() << endl;
    cout << "pqv  R    : \n" << pqv.q().matrix() << endl;
    cout << "pqv  R*vel: " << (pqv.q().matrix() * pqv.v()).transpose() << endl;
    cout << "pqv  q*vel: " << (pqv.q() * pqv.v()).transpose() << endl;
    cout << "pqv  state: " << pqv.x().transpose() << endl;
    cout << "storage   : " << storage.transpose() << endl;
    cout << "doing 'storage *= 2'" << endl;
    storage *= 2;
    cout << "quat norm : " << pqv.q().norm() << endl;
    cout << "doing 'pqv.q().setIdentity()'" << endl;
    pqv.q().setIdentity();
    cout << "pqv  quat : " << pqv.q().coeffs().transpose();
    cout << endl;

    cout << "\n---Testing local and remote states---" << endl;
    cout << "imustate  : " << imustate.transpose() << endl;
    StateIMU imulocal(imustate);
    cout << "imulocal  : " << imulocal.x().transpose() << endl;
    cout << "imulocal ab, wb: " << imulocal.ab().transpose() << " , " << imulocal.wb().transpose() << endl;
    cout << "imu       : " << imu.x().transpose() << endl;
    cout << "doing 'imulocal.abias() *= 3'" << endl;
    imulocal.ab() *= 3;
    cout << "imulocal  : " << imulocal.x().transpose() << endl;
    cout << "imu       : " << imu.x().transpose() << endl;
    cout << "doing imu = imulocal" << endl;
    imu = imulocal;
    cout << "imu       : " << imu.x().transpose() << endl;
    cout << "storage   : " << storage.transpose() << endl;
    StatePQV pqvlocal(pqvstate);
    cout << "pqvlocal  : " << pqvlocal.x().transpose() << endl;
    cout << "pqvlocal p, v, q: " << pqvlocal.p().transpose() << " , " << pqvlocal.v().transpose() << " , "
            << pqvlocal.q().coeffs().transpose() << endl;
    cout << "pqv       : " << pqv.x().transpose() << endl;
    cout << "doing 'pqvlocal.v() *= 2'" << endl;
    pqvlocal.v() *= 2;
    cout << "pqvlocal  : " << pqvlocal.x().transpose() << endl;
    cout << "pqv       : " << pqv.x().transpose() << endl;
    cout << "doing pqv = pqvlocal" << endl;
    pqv = pqvlocal;
    cout << "pqv       : " << pqv.x().transpose() << endl;
    cout << "storage   : " << storage.transpose() << endl;

    // Miscelaneous
    Quaternions q(1, 2, 3, 4);
    cout << "\nAttention!!! constructing Quaternions q(1,2,3,4) gives coeffs order: " << q.coeffs().transpose();
    cout << endl;

//    cout << "\n--- Test composite state StatePQVBB, with PQV and IMU ---" << endl;
//
//    cout << "--- 1. With local states ---" << endl;
//    StateCompPQVBB pqvbblocal(pqvstate, imustate);
//    cout << "storage   : " << storage.transpose() << endl;
//    cout << "pqvbblocal p  : " << pqvbblocal.p().transpose() << endl;
//    cout << "pqvbblocal v  : " << pqvbblocal.v().transpose() << endl;
//    cout << "pqvbblocal q  : " << pqvbblocal.q().coeffs().transpose() << endl;
//    cout << "pqvbblocal ab : " << pqvbblocal.abias().transpose() << endl;
//    cout << "pqvbblocal wb : " << pqvbblocal.wbias().transpose() << endl;
//    cout << "pqvbblocal pqv: " << pqvbblocal.StatePQV::x().transpose() << endl;
//    cout << "pqvbblocal bb : " << pqvbblocal.StateIMU::x().transpose() << endl;
//
//    cout << "--- 2. With remote states ---" << endl;
//    StateCompPQVBB pqvbb(storage, index + bsize, index + bsize + StatePQV::SIZE_, pqvstate, imustate);
//    cout << "storage   : " << storage.transpose() << endl;
//    cout << "doing 'storage *= 2'" << endl;
//    storage *= 2;
//    cout << "storage   : " << storage.transpose() << endl;
//    cout << "pqvbb p  : " << pqvbb.p().transpose() << endl;
//    cout << "pqvbb v  : " << pqvbb.v().transpose() << endl;
//    cout << "pqvbb q  : " << pqvbb.q().coeffs().transpose() << endl;
//    cout << "pqvbb ab : " << pqvbb.abias().transpose() << endl;
//    cout << "pqvbb wb : " << pqvbb.wbias().transpose() << endl;
//    cout << "pqvbb pqv: " << pqvbb.StatePQV::x().transpose() << endl;
//    cout << "pqvbb bb : " << pqvbb.StateIMU::x().transpose() << endl;
//
//    cout << "pqvbb     SIZE_ : " << pqvbb.SIZE_ << endl;
//    cout << "pqvbb pqv SIZE_ : " << pqvbb.StatePQV::SIZE_ << endl;
//    cout << "pqvbb bb  SIZE_ : " << pqvbb.StateIMU::SIZE_ << endl;

    cout << "\n---Testing local error states---" << endl;
    VectorXs pvqerror;
    pvqerror = VectorXs::Constant(StateErrorPQV::SIZE_ERROR_, 0.001);
    StateErrorPQV epqv( pqvstate );
    epqv.qn().normalize();
    epqv.xe() = pvqerror;
    cout << "epvq pn: " << epqv.pn().transpose() << endl;
    cout << "epvq qn: " << epqv.qn().coeffs().transpose() << endl;
    cout << "epvq vn: " << epqv.vn().transpose() << endl;
    cout << "epvq pe: " << epqv.pe().transpose() << endl;
    cout << "epvq qe: " << epqv.qe().transpose() << endl;
    cout << "epvq ve: " << epqv.ve().transpose() << endl;
    cout << "epvq pc: " << epqv.pc().transpose() << endl;
    cout << "epvq qc: " << epqv.qc().coeffs().transpose() << endl;
    cout << "epvq vc: " << epqv.vc().transpose() << endl;

    cout << "----------------------------" << endl;
    return 0;
}

