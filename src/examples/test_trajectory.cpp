#include <iostream>
// #include "wolf.h"
#include "trajectory.h"

using namespace std;
using namespace Eigen;

int main()
{
    cout << "\nTrajectory of StatePQV and StateIMU - demo";
    cout << "\n------------------------------------------\n" << endl;
    unsigned int length = 4;
    unsigned int x_size = length*StatePQV::SIZE_+StateIMU::SIZE_;
    TrajectoryPQVIMU trj(length);
    trj.x() = VectorXs::LinSpaced(x_size, 0, x_size-1);
    cout << "Full state   : " << trj.x().transpose() << endl;

    VectorXs pqv(StatePQV::SIZE_);
    pqv = VectorXs::Constant(StatePQV::SIZE_, 0);

    VectorXs bb(StateIMU::SIZE_);
    bb = VectorXs::Constant(StateIMU::SIZE_, -1);

    trj.otherState().x(bb);

    cout << trj << endl;

    for (unsigned int i = 0; i < length + 3; i++)
    {
        pqv.array() += 2; // modify state with this fake odometry
        StatePQV pqv_tmp(pqv);
        Frame<StatePQV> fr(i, pqv_tmp);
        fr.setKey();
        trj.addFrame(fr);

        cout << trj << endl;
    }

    trj.completeUnprocessedFrames();

    cout << "Full state   : " << trj.x().transpose() << endl;

    cout << "Head  : (p, v, q) = (";
    cout << trj.frame(0).state().p().transpose() << ", ";
    cout << trj.frame(0).state().v().transpose() << ", ";
    cout << trj.frame(0).state().q().coeffs().transpose() << ")" << endl;

    cout << "Second: (p, v, q) = (";
    cout << trj.frame(1).state().p().transpose() << ", ";
    cout << trj.frame(1).state().v().transpose() << ", ";
    cout << trj.frame(1).state().q().coeffs().transpose() << ")" << endl;

    cout << "Tail  : (p, v, q) = (";
    cout << trj.frame(length-1).state().p().transpose() << ", ";
    cout << trj.frame(length-1).state().v().transpose() << ", ";
    cout << trj.frame(length-1).state().q().coeffs().transpose() << ")" << endl;

    cout << "Other : (ab, wb) = (";
    cout << trj.otherState().ab().transpose() << ", ";
    cout << trj.otherState().wb().transpose() << ")" << endl;


    return 0;
}

