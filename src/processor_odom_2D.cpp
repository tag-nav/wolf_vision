#include "processor_odom_2D.h"
namespace wolf
{

ProcessorBase* ProcessorOdom2D::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorOdom2D* prc_ptr = new ProcessorOdom2D();
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

}


// Register in the ProcessorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
const bool registered_prc_odom_2d = ProcessorFactory::get()->registerCreator("ODOM 2D", ProcessorOdom2D::create);
}
} // namespace wolf
