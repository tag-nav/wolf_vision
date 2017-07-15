#include "processor_odom_2D.h"
namespace wolf
{

ProcessorBasePtr ProcessorOdom2D::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr)
{
    Scalar dist_traveled_th ;
    Scalar cov_det_th       ;
    Scalar elapsed_time_th  ;
    Scalar unmeasured_perturbation_std;
    if (_params)
    {
        std::shared_ptr<ProcessorParamsOdom2D> params = std::static_pointer_cast<ProcessorParamsOdom2D>(_params);
        dist_traveled_th = params->dist_traveled_th_;
        cov_det_th       = params->cov_det_th_;
        elapsed_time_th  = params->elapsed_time_th_;
        unmeasured_perturbation_std = params->unmeasured_perturbation_std_;
    }
    else
    {
        std::cout << __FILE__ << ":" << __FUNCTION__ << "() : No parameters provided. Using dummy set." << std::endl;
        dist_traveled_th = 1;
        cov_det_th       = 1;
        elapsed_time_th  = 1;
        unmeasured_perturbation_std = 1;
    }
    ProcessorOdom2DPtr prc_ptr = std::make_shared<ProcessorOdom2D>(dist_traveled_th, cov_det_th, elapsed_time_th, unmeasured_perturbation_std);
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

}


// Register in the ProcessorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("ODOM 2D", ProcessorOdom2D)
} // namespace wolf
