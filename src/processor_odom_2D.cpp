#include "processor_odom_2D.h"
namespace wolf
{

ProcessorBasePtr ProcessorOdom2D::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr)
{

    ProcessorOdom2DPtr prc_ptr;

    if (_params)
    {
        std::shared_ptr<ProcessorParamsOdom2D> params = std::static_pointer_cast<ProcessorParamsOdom2D>(_params);

        prc_ptr = std::make_shared<ProcessorOdom2D>(params->dist_traveled_th_,
                                                    params->theta_traveled_th_,
                                                    params->cov_det_th_,
                                                    params->elapsed_time_th_,
                                                    params->unmeasured_perturbation_std_);
    }
    else
    {
        std::cout << __FILE__ << ":" << __FUNCTION__ << "() : No parameters provided. Using default set." << std::endl;

        prc_ptr = std::make_shared<ProcessorOdom2D>();
    }

    prc_ptr->setName(_unique_name);

    return prc_ptr;
}

}


// Register in the ProcessorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("ODOM 2D", ProcessorOdom2D)
} // namespace wolf
