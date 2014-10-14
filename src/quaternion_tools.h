/*
 * quaternion_tools.h
 *
 *  Created on: 01/06/2014
 *      \author: jsola
 */

#ifndef QUATERNION_TOOLS_H_
#define QUATERNION_TOOLS_H_

//eigen
#include <eigen3/Eigen/Geometry>
#include "wolf.h"



//XXX : inherit from Eigen::Quaternion and add this method, or extend Quaternion<> somehow?
namespace Wolf{

using namespace std;
using namespace Eigen;

/**
 * Quaternion from rotation vector
 */
template<class V3>
Quaternions quaternionFromVector(const V3& rvec)
{
    assert (rvec.size() == 3);
    if (rvec.isMuchSmallerThan(0,1e-8))
        return Quaternions::Identity();
    else
    {
//        Quaternions q(AngleAxiss(rvec.norm(), rvec.normalized()));
        return  Quaternions(AngleAxiss(rvec.norm(), rvec.normalized()));
//        return q;
    }
}

}


#endif /* QUATERNION_TOOLS_H_ */
