/**
 * \file processor_odom_2D.h
 *
 *  Created on: Apr 15, 2016
 *      \author: jvallve
 */

#ifndef SRC_PROCESSOR_ODOM_2D_H_
#define SRC_PROCESSOR_ODOM_2D_H_

#include "processor_motion.h"


namespace wolf {

/**\brief The Motion Delta type
 *
 * The motion delta, as a composite struct containing position increment and orientation quaternion increment.
 */
struct Odom2dDelta{
        Eigen::Vector2s dp;    ///< Position increment
        WolfScalar dq; ///< Orientation increment as a quaternion
        Odom2dDelta() : dp(0,0), dq(0){};
        Odom2dDelta(const Eigen::Vector2s& _dp, const WolfScalar& _dq) : dp(_dp), dq(_dq) {};
        void set(Eigen::Vector2s _dp, WolfScalar _dq){dp = _dp; dq = _dq;};
        void setZero() {dp.setZero(); dq=0;}
        static Odom2dDelta Zero() {return Odom2dDelta();}
};



class ProcessorOdom2d : public ProcessorMotion
{
    public:
        ProcessorOdom2d(WolfScalar _delta_t = 0);
        virtual ~ProcessorOdom2d();
        virtual void data2delta(const Eigen::VectorXs& _data, const WolfScalar _dt, Eigen::VectorXs& _delta);

    protected:
        virtual void preProcess(){}
        virtual void postProcess(){}

    private:
        void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta);
        void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2);
        virtual void deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                     Eigen::VectorXs& _delta2_minus_delta1);
        Eigen::VectorXs deltaZero() const;
};


inline ProcessorOdom2d::ProcessorOdom2d(WolfScalar _delta_t) :
        ProcessorMotion(PRC_ODOM_2D, _delta_t, 3, 3, 2)
{
}

inline ProcessorOdom2d::~ProcessorOdom2d()
{
}

inline void ProcessorOdom2d::data2delta(const Eigen::VectorXs& _data, const WolfScalar _dt, Eigen::VectorXs& _delta)
{
    assert(_delta.size() == 3 && "Wrong _delta vector size");
    assert(_data.size() == 2 && "Wrong _data vector size");

    // 1/2 turn + straight + 1/2 turn
    _delta(0) = cos(_data(1)/2) * _data(0);
    _delta(1) = sin(_data(1)/2) * _data(0);
    _delta(2) = _data(1);
}

inline void ProcessorOdom2d::xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta)
{
    assert(_x.size() == 3 && "Wrong _x vector size");
    assert(_x_plus_delta.size() == 3 && "Wrong _x_plus_delta vector size");

//    std::cout << "xPlusDelta ------------------------------------" << std::endl;
//    std::cout << "_x:     " << _x.transpose() << std::endl;
//    std::cout << "_delta: " << _delta.transpose() << std::endl;
//    std::cout << "_x_plus_delta: " << _x_plus_delta.transpose() << std::endl;

    _x_plus_delta.head<2>() = _x.head<2>() + Eigen::Rotation2Ds(_x(2)).matrix() * _x_plus_delta.head<2>();
    _x_plus_delta(2) = _x(2) + _x_plus_delta(2);

//    std::cout << "-----------------------------------------------" << std::endl;
//    std::cout << "_x_plus_delta: " << _x_plus_delta.transpose() << std::endl;
}

inline void ProcessorOdom2d::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2)
{
    assert(_delta1.size() == 3 && "Wrong _delta1 vector size");
    assert(_delta2.size() == 3 && "Wrong _delta2 vector size");

//    std::cout << "xPlusDelta ------------------------------------" << std::endl;
//    std::cout << "_delta1: " << _delta1.transpose() << std::endl;
//    std::cout << "_delta2: " << _delta2.transpose() << std::endl;
//    std::cout << "_delta1_plus_delta2: " << _delta1_plus_delta2.transpose() << std::endl;

    _delta1_plus_delta2.head<2>() = _delta1.head<2>() + Eigen::Rotation2Ds(_delta1(2)).matrix() * _delta2.head<2>();
    _delta1_plus_delta2(2) = _delta1(2) + _delta2(2);

//    std::cout << "-----------------------------------------------" << std::endl;
//    std::cout << "_delta1_plus_delta2: " << _delta1_plus_delta2.transpose() << std::endl;
}

inline void ProcessorOdom2d::deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                             Eigen::VectorXs& _delta2_minus_delta1)
{
    assert(_delta1.size() == 3 && "Wrong _delta1 vector size");
    assert(_delta2.size() == 3 && "Wrong _delta2 vector size");

//    std::cout << "xPlusDelta ------------------------------------" << std::endl;
//    std::cout << "_delta1: " << _delta1.transpose() << std::endl;
//    std::cout << "_delta2: " << _delta2.transpose() << std::endl;
//    std::cout << "_delta2_minus_delta1: " << _delta2_minus_delta1.transpose() << std::endl;

    _delta2_minus_delta1.head<2>() =  Eigen::Rotation2Ds(-_delta1(2)).matrix() * (_delta2.head<2>() - _delta1.head<2>());
    _delta2_minus_delta1(2) = _delta2(2) - _delta1(2);

//    std::cout << "-----------------------------------------------" << std::endl;
//    std::cout << "_delta2_minus_delta1: " << _delta2_minus_delta1.transpose() << std::endl;
}

inline Eigen::VectorXs ProcessorOdom2d::deltaZero() const
{
    return Eigen::VectorXs::Zero(3);
}

} // namespace wolf

#endif /* SRC_PROCESSOR_ODOM_2D_H_ */
