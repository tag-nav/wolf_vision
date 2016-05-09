#include "processor_odom_3D.h"
namespace wolf
{

ProcessorBase* ProcessorOdom3D::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorOdom3D* prc_ptr = new ProcessorOdom3D();
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

}


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
const bool registered_prc_odom_3d = ProcessorFactory::get()->registerCreator("ODOM 3D", ProcessorOdom3D::create);
}
} // namespace wolf
