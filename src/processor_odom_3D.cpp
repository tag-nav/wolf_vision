#include "processor_odom_3D.h"
namespace wolf
{

ProcessorOdom3D::ProcessorOdom3D() :
        ProcessorMotion(PRC_ODOM_3D, "ODOM 3D", 7, 7, 6, 6),
        k_disp_to_disp_(0.1), k_disp_to_rot_(0.1), k_rot_to_rot_(0.1),
        p1_(nullptr), p2_(nullptr), p_out_(nullptr),
        q1_(nullptr), q2_(nullptr), q_out_(nullptr)
{
    jacobian_delta_preint_.setIdentity(delta_cov_size_, delta_cov_size_);
    jacobian_delta_.setIdentity(delta_cov_size_, delta_cov_size_);
}

ProcessorOdom3D::~ProcessorOdom3D()
{
}

ProcessorBasePtr ProcessorOdom3D::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params)
{
    std::shared_ptr<ProcessorOdom3D> prc_ptr = std::make_shared<ProcessorOdom3D>();
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

}


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("ODOM 3D", ProcessorOdom3D)
} // namespace wolf
