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
    MatrixXs M(4,3);

    Map<VectorXs> q0(&x(0),4);
    q0.setRandom();
    q0.normalize();
    Map<VectorXs> da(&x(4),3);
    da << .1,.2,.3;
    Map<VectorXs> q1(&x(7),4);
    Map<MatrixXs> J(M.data(),4,3);

    cout << "x=" << x.transpose() << endl;
    cout << "q0=" << q0.transpose() << "\nda= " << da.transpose() << "\nq1=" << q1.transpose() << endl;

    LocalParametrizationQuaternion Q_param;

    Q_param.plus(q0,da,q1);
    cout << "q1 = " << q1.transpose() << endl;

    Q_param.computeJacobian(q0,J);
    cout << "J=" << J << endl;


    return 0;
}
