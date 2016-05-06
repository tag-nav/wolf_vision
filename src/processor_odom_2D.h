/**
 * \file processor_odom_2D.h
 *
 *  Created on: Apr 15, 2016
 *      \author: jvallve
 */

#ifndef SRC_PROCESSOR_ODOM_2D_H_
#define SRC_PROCESSOR_ODOM_2D_H_

#include "processor_motion.h"
#include "constraint_odom_2D.h"


namespace wolf {

class ProcessorOdom2D : public ProcessorMotion
{
    public:
        ProcessorOdom2D();
        virtual ~ProcessorOdom2D();
        virtual void data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt,
                                Eigen::VectorXs& _delta, Eigen::MatrixXs& _delta_cov);

    protected:
//        virtual void preProcess(){}
//        virtual void postProcess(){}

    private:
        void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta);
        void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2);
        void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                            Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                            Eigen::MatrixXs& _jacobian2);
        virtual void deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                     Eigen::VectorXs& _delta2_minus_delta1);
        Eigen::VectorXs deltaZero() const;
        Motion interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts);

        virtual ConstraintBase* createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin);

        // Factory method
        public:
            static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};


inline ProcessorOdom2D::ProcessorOdom2D() :
        ProcessorMotion(PRC_ODOM_2D, 3, 3, 2)
{
    setType("ODOM 2D");
}

inline ProcessorOdom2D::~ProcessorOdom2D()
{
}

inline void ProcessorOdom2D::data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt,
                                        Eigen::VectorXs& _delta, Eigen::MatrixXs& _delta_cov)
{
    //std::cout << "ProcessorOdom2d::data2delta" << std::endl;

    assert(_delta.size() == delta_size_ && "Wrong _delta vector size");
    assert(_delta_cov.rows() == delta_size_ && "Wrong _delta_cov size");
    assert(_delta_cov.cols() == delta_size_ && "Wrong _delta_cov size");
    assert(_data.size() == data_size_ && "Wrong _data vector size");
    assert(_data_cov.rows() == data_size_ && "Wrong _data_cov size");
    assert(_data_cov.cols() == data_size_ && "Wrong _data_cov size");

    // 1/2 turn + straight + 1/2 turn
    _delta(0) = cos(_data(1)/2) * _data(0);
    _delta(1) = sin(_data(1)/2) * _data(0);
    _delta(2) = _data(1);

    // Fill delta covariance
    Eigen::MatrixXs J(3,2);
    J(0,0) = cos(_data(1) / 2);
    J(1,0) = sin(_data(1) / 2);
    J(2,0) = 0;
    J(0,1) =-_data(0) / 2 * sin(_data(1) / 2);
    J(1,1) = _data(0) / 2 * cos(_data(1) / 2);
    J(2,1) = 1;

    _delta_cov = J * _data_cov * J.transpose();

    //std::cout << "data cov:" << std::endl << _data_cov << std::endl;
    //std::cout << "delta cov:" << std::endl << _delta_cov << std::endl;
}

inline void ProcessorOdom2D::xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta)
{
    //std::cout << "ProcessorOdom2d::xPlusDelta" << std::endl;

    assert(_x.size() == x_size_ && "Wrong _x vector size");
    assert(_x_plus_delta.size() == delta_size_ && "Wrong _x_plus_delta vector size");

//    std::cout << "xPlusDelta ------------------------------------" << std::endl;
//    std::cout << "_x:     " << _x.transpose() << std::endl;
//    std::cout << "_delta: " << _delta.transpose() << std::endl;
//    std::cout << "_x_plus_delta: " << _x_plus_delta.transpose() << std::endl;

    _x_plus_delta.head<2>() = _x.head<2>() + Eigen::Rotation2Ds(_x(2)).matrix() * _delta.head<2>();
    _x_plus_delta(2) = _x(2) + _delta(2);

//    std::cout << "-----------------------------------------------" << std::endl;
//    std::cout << "_x_plus_delta: " << _x_plus_delta.transpose() << std::endl;
}

inline void ProcessorOdom2D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2)
{
    //std::cout << "ProcessorOdom2d::deltaPlusDelta" << std::endl;
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");

//    std::cout << "deltaPlusDelta ------------------------------------" << std::endl;
//    std::cout << "_delta1: " << _delta1.transpose() << std::endl;
//    std::cout << "_delta2: " << _delta2.transpose() << std::endl;
//    std::cout << "_delta1_plus_delta2: " << _delta1_plus_delta2.transpose() << std::endl;

    _delta1_plus_delta2.head<2>() = _delta1.head<2>() + Eigen::Rotation2Ds(_delta1(2)).matrix() * _delta2.head<2>();
    _delta1_plus_delta2(2) = _delta1(2) + _delta2(2);

//    std::cout << "-----------------------------------------------" << std::endl;
//    std::cout << "_delta1_plus_delta2: " << _delta1_plus_delta2.transpose() << std::endl;
}

inline void ProcessorOdom2D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                            Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                            Eigen::MatrixXs& _jacobian2)
{
    //std::cout << "ProcessorOdom2d::deltaPlusDelta jacobians" << std::endl;
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");
    assert(_jacobian1.rows() == delta_size_ && "Wrong _jacobian1 size");
    assert(_jacobian1.cols() == delta_size_ && "Wrong _jacobian1 size");
    assert(_jacobian2.rows() == delta_size_ && "Wrong _jacobian2 size");
    assert(_jacobian2.cols() == delta_size_ && "Wrong _jacobian2 size");

//    std::cout << "deltaPlusDelta ------------------------------------" << std::endl;
//    std::cout << "_delta1: " << _delta1.transpose() << std::endl;
//    std::cout << "_delta2: " << _delta2.transpose() << std::endl;
//    std::cout << "_delta1_plus_delta2: " << _delta1_plus_delta2.transpose() << std::endl;

    _delta1_plus_delta2.head<2>() = _delta1.head<2>() + Eigen::Rotation2Ds(_delta1(2)).matrix() * _delta2.head<2>();
    _delta1_plus_delta2(2) = _delta1(2) + _delta2(2);

    // TODO: fill the jacobians
    _jacobian1 = Eigen::MatrixXs::Identity(delta_size_,delta_size_);
    _jacobian1(0,2) = -sin(_delta1(2))*_delta2(0) - cos(_delta1(2))*_delta2(1);
    _jacobian1(1,2) =  cos(_delta1(2))*_delta2(0) - sin(_delta1(2))*_delta2(1);
    _jacobian2 = Eigen::MatrixXs::Identity(delta_size_,delta_size_);
    _jacobian2.topLeftCorner<2,2>() = Eigen::Rotation2Ds(_delta1(2)).matrix();

    //std::cout << "-----------------------------------------------" << std::endl;
    //std::cout << "_delta1_plus_delta2: " << _delta1_plus_delta2.transpose() << std::endl;
}

inline void ProcessorOdom2D::deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                             Eigen::VectorXs& _delta2_minus_delta1)
{
    //std::cout << "ProcessorOdom2d::deltaMinusDelta" << std::endl;
    assert(_delta1.size() == 3 && "Wrong _delta1 vector size");
    assert(_delta2.size() == 3 && "Wrong _delta2 vector size");
    assert(_delta2_minus_delta1.size() == delta_size_ && "Wrong _delta2_minus_delta1 vector size");

//    std::cout << "deltaMinusDelta ------------------------------------" << std::endl;
//    std::cout << "_delta1: " << _delta1.transpose() << std::endl;
//    std::cout << "_delta2: " << _delta2.transpose() << std::endl;
//    std::cout << "_delta2_minus_delta1: " << _delta2_minus_delta1.transpose() << std::endl;

    _delta2_minus_delta1.head<2>() =  Eigen::Rotation2Ds(-_delta1(2)).matrix() * (_delta2.head<2>() - _delta1.head<2>());
    _delta2_minus_delta1(2) = _delta2(2) - _delta1(2);

//    std::cout << "-----------------------------------------------" << std::endl;
//    std::cout << "_delta2_minus_delta1: " << _delta2_minus_delta1.transpose() << std::endl;
}

inline Eigen::VectorXs ProcessorOdom2D::deltaZero() const
{
    return Eigen::VectorXs::Zero(delta_size_);
}

inline ConstraintBase* ProcessorOdom2D::createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin)
{
    return new ConstraintOdom2D(_feature_motion, _frame_origin);
}

inline Motion ProcessorOdom2D::interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts)
{
    Motion tmp(_motion_ref);
    tmp.ts_ = _ts;
    tmp.delta_ = deltaZero();
    tmp.delta_cov_ = Eigen::MatrixXs::Zero(delta_size_, delta_size_);
    return tmp;
}

ProcessorBase* ProcessorOdom2D::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorOdom2D* prc_ptr = new ProcessorOdom2D();
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

} // namespace wolf



// Register in the ProcessorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
const bool registered_prc_odom_2d = ProcessorFactory::get()->registerCreator("ODOM 2D", ProcessorOdom2D::create);
}
} // namespace wolf



#endif /* SRC_PROCESSOR_ODOM_2D_H_ */
