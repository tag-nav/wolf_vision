/*
 * \test_homogeneous_param.cpp
 *
 *  Created on: 25/02/2016
 *      \author: jsola
 */


/*
 * \file test_local_param_quat.cpp
 *
 *  Created on: Feb 19, 2016
 *      author: jsola
 */


#include "local_parametrization_homogeneous.h"

#include "wolf.h"

#include <iostream>

int main(){

    using namespace Eigen;
    using namespace std;

    VectorXs x(11);
    MatrixXs M(4,3);

    Map<VectorXs> h(&x(0),4);
    h.setRandom();
//    h.normalize();
    Map<VectorXs> d(&x(4),3);
    d << .1,.2,.3;
    Map<VectorXs> h_out(&x(7),4);
    Map<MatrixXs> J(M.data(),4,3);

    cout << "\n x = " << x.transpose() << endl;
    cout << "\nq0 = " << h.transpose() << "\nda = " << d.transpose() << endl;

    LocalParametrizationHomogeneous Hpar;

    Hpar.plus(h,d,h_out);
    cout << "\nq_out = " << h_out.transpose() << endl;

    Hpar.computeJacobian(h,J);
    cout << " J = " << J << "\n" << endl;


    return 0;
}


