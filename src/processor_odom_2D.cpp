#include "processor_odom_2D.h"
namespace wolf
{

ProcessorBasePtr ProcessorOdom2D::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params)
{
    ProcessorParamsOdom2D* params = (ProcessorParamsOdom2D*)_params;
    ProcessorOdom2D* prc_ptr = new ProcessorOdom2D(params->dist_traveled_th_, params->cov_det_th_, params->elapsed_time_th_);
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

}


// Register in the ProcessorFactory
#include "processor_factory.h"
namespace wolf {
//namespace
//{
//const bool registered_prc_odom_2d = ProcessorFactory::get().registerCreator("ODOM 2D", ProcessorOdom2D::create);
WOLF_REGISTER_PROCESSOR("ODOM 2D", ProcessorOdom2D)
//}
} // namespace wolf
