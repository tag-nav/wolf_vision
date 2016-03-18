/*
 * \file test_local_param_quat.cpp
 *
 *  Created on: Feb 19, 2016
 *      author: jsola
 */


#include "local_parametrization_quaternion.h"
#include "local_parametrization_homogeneous.h"

#include "wolf.h"

#include <iostream>

int main(){

    using namespace Eigen;
    using namespace std;

    VectorXs x(22);
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

    cout << "\nGLOBAL D_QUAT plus()" << endl;
    Map<const VectorXs> q_const(q.data(),4);
    Map<const VectorXs> da_const(da.data(),3);
    Qpar.plus(q_const,da_const,qo);
    cout << "qo = " << qo.transpose() << "   with norm = " << qo.norm() << endl;

    Qpar.computeJacobian(q_const,J);
    cout << " J = " << J << endl << endl;

    cout << "\nLOCAL D_QUAT plus()" << endl;
    Qpar_loc.plus(q_const,da_const,qo);
    cout << "qo = " << qo.transpose() << "   with norm = " << qo.norm() << endl;

    Qpar_loc.computeJacobian(q_const,J);
    cout << " J = " << J << endl;

    cout << "\nHOMOGENEOUS plus()" << endl;
    Map<VectorXs> h(&x(11),4);
    h.setRandom();
    Map<VectorXs> d(&x(15),3);
    d << .1,.2,.3;
    Map<VectorXs> h_out(&x(18),4);

    cout << "Initial values:" << endl;
    cout << "h  = " << h.transpose() << "   with norm: " << h.norm() << endl;
    cout << "d  = " << d.transpose() << endl;

    LocalParametrizationHomogeneous Hpar;
    Map<const VectorXs> h_const(h.data(),4);
    Map<const VectorXs> d_const(d.data(),3);

    Hpar.plus(h_const,d_const,h_out);
    cout << "\nh_out = " << h_out.transpose() << "   with norm: " << h_out.norm() << endl;

    Hpar.computeJacobian(h_const,J);
    cout << " J = " << J << "\n" << endl;


    return 0;
}
