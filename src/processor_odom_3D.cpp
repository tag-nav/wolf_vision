#include "processor_odom_3D.h"
namespace wolf
{

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
