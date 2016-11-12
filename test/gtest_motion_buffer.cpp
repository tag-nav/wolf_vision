/*
 * gtest_motion_buffer.cpp
 *
 *  Created on: Nov 12, 2016
 *      Author: jsola
 */


#include "utils_gtest.h"
#include "../src/logging.h"

#include "../src/motion_buffer.h"

#include <iostream>

using namespace Eigen;
using namespace std;
using namespace wolf;

Motion newMotion(TimeStamp t, Scalar d, Scalar D, Scalar c, Scalar C)
{
    Motion m(t, 1);
    m.delta_(0) = d;
    m.delta_integr_(0) = D;
    m.delta_cov_(0) = c;
    m.delta_integr_cov_(0) = C;
    return m;
}

namespace{
TimeStamp t0(0), t1(1), t2(2), t3(3), t4(4);
Motion m0 = newMotion(t0,0,0,0,0);
Motion m1 = newMotion(t1,1,1,1,1);
Motion m2 = newMotion(t2,2,3,1,2);
Motion m3 = newMotion(t3,3,6,1,3);
Motion m4 = newMotion(t4,4,10,1,4);


}

TEST(MotionBuffer, QueryTimeStamps)
{
    MotionBuffer MB;

    MB.get().push_back(m0);
    MB.get().push_back(m1);
    TimeStamp t;

    t.set(-1); // t is older than m0.ts_ -> return m0
    ASSERT_EQ(MB.getMotion(t).ts_ , m0.ts_);

    t.set( 0); // t is exactly m0.ts_ -> return m0
    ASSERT_EQ(MB.getMotion(t).ts_ , m0.ts_);

    t.set(0.5); // t is between m0.ts_ and m1.ts_ -> return m0
    ASSERT_EQ(MB.getMotion(t).ts_ , m0.ts_);

    t.set(+1); // t is exactly m1.ts_ -> return m1
    ASSERT_EQ(MB.getMotion(t).ts_ , m1.ts_);

    t.set(+2); // t is newer than m1.ts_ -> return m1
    ASSERT_EQ(MB.getMotion(t).ts_ , m1.ts_);
}

TEST(MotionBuffer, getMotion)
{
    MotionBuffer MB;

    MB.get().push_back(m0);
    ASSERT_EQ(MB.getMotion(t0).delta_, m0.delta_);

    MB.get().push_back(m1);
    ASSERT_EQ(MB.getMotion(t0).delta_, m0.delta_);
    ASSERT_EQ(MB.getMotion(t1).delta_, m1.delta_);
    ASSERT_EQ(MB.getMotion(t0).delta_integr_, m0.delta_integr_);
    ASSERT_EQ(MB.getMotion(t1).delta_integr_, m1.delta_integr_);
}

TEST(MotionBuffer, getDelta)
{
    MotionBuffer MB;

    MB.get().push_back(m0);

    ASSERT_EQ(MB.getDelta(t0), m0.delta_integr_);

    MB.get().push_back(m1);

    ASSERT_EQ(MB.getDelta(t0), m0.delta_integr_);
    ASSERT_EQ(MB.getDelta(t1), m1.delta_integr_);
}

TEST(MotionBuffer, Split)
{
    MotionBuffer MB;

    MB.get().push_back(m0);
    MB.get().push_back(m1);
    MB.get().push_back(m2);
    MB.get().push_back(m3);
    MB.get().push_back(m4); // put 5 motions

    MotionBuffer MB_old;

    TimeStamp t;

    t = 1.5; // between m1 and m2
    MB.split(t, MB_old);

    ASSERT_EQ(MB_old.get().size(),      2);
    ASSERT_EQ(MB    .get().size(),      3);

    ASSERT_EQ(MB_old.getMotion(t1).ts_, t1);
    ASSERT_EQ(MB_old.getMotion(t2).ts_, t1); // last  ts is t1
    ASSERT_EQ(MB    .getMotion(t1).ts_, t2); // first ts is t2
    ASSERT_EQ(MB    .getMotion(t2).ts_, t2);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


