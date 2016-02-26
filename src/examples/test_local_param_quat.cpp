/*
 * \file test_local_param_quat.cpp
 *
 *  Created on: Feb 19, 2016
 *      author: jsola
 */


#include "local_parametrization_quaternion.h"

#include "wolf.h"

#include <iostream>

int main(){

    using namespace Eigen;
    using namespace std;

    VectorXs x(11);
    MatrixXs M(1,12); // matrix dimensions do not matter, just storage size.

    x.setRandom();
    M.setZero();

    Map<VectorXs> q(&x(0),4);
    q.normalize();

    Map<VectorXs> da(&x(4),3);
    da /= 10.0;
    Map<VectorXs> qo(&x(7),4);
    Map<MatrixXs> J(&M(0,0),4,3);

    cout << "Initial values:" << endl;
    cout << "q  = " << q.transpose() << "   with norm = " << q.norm() << "\nda = " << da.transpose() << endl;
    cout << "qo = " << qo.transpose() << "   with norm = " << qo.norm() << endl;

    LocalParametrizationQuaternion Qpar;
    LocalParametrizationQuaternion Qpar_loc(DQ_LOCAL);

    cout << "GLOBAL plus()\n" << endl;
    Qpar.plus(q,da,qo);
    cout << "qo = " << qo.transpose() << "   with norm = " << qo.norm() << endl;

    Qpar.computeJacobian(q,J);
    cout << " J = " << J << endl << endl;

    cout << "LOCAL plus()\n" << endl;
    Qpar_loc.plus(q,da,qo);
    cout << "qo = " << qo.transpose() << "   with norm = " << qo.norm() << endl;

    Qpar_loc.computeJacobian(q,J);
    cout << " J = " << J << endl;

    return 0;
}
