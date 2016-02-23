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

    Map<VectorXs> q(&x(0),4);
    q.setRandom();
//    q.normalize();

    Map<VectorXs> da(&x(4),3);
    da << .1,.2,.3;
    Map<VectorXs> q_out(&x(7),4);
    Map<MatrixXs> J(M.data(),4,3);

    cout << "\nq0 = " << q.transpose() << "\nda = " << da.transpose() << endl;

    LocalParametrizationQuaternion Qpar;

    Qpar.plus(q,da,q_out);
    cout << "\nq_out = " << q_out.transpose() << endl;

    Qpar.computeJacobian(q,J);
    cout << "\n J = " << J << "\n" << endl;


    return 0;
}
