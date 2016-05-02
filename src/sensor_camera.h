#ifndef SENSOR_CAMERA_H
#define SENSOR_CAMERA_H

//wolf includes
#include "sensor_base.h"
#include "sensor_factory.h"


namespace wolf {

/** Struct of all intrinsic camera parameters
 */
struct IntrinsicsCamera : public IntrinsicsBase
{
        unsigned int width;                 // Image width in pixels
        unsigned int height;                // Image height in pixels
        Eigen::Vector4s intrinsic_vector;   // k = [u_0, v_0, alpha_u, alpha_v]  vector of pinhole-model intrinsic parameters
        Eigen::VectorXs distortion;         // d = [d_1, d_2, d_3, ...] radial distortion coefficients
        Eigen::VectorXs correction;         // c = [c_1, c_2, c_3, ...] radial distortion correction coefficients
};

/**Pin-hole camera sensor
 */
class SensorCamera : public SensorBase
{
    public:
    /** \brief Constructor with arguments
     *
     * Constructor with arguments
     * \param _p_ptr StateBlock pointer to the sensor position wrt vehicle base
     * \param _o_ptr StateBlock pointer to the sensor orientation wrt vehicle base
     * \param _intr_ptr contains intrinsic values for the camera
     * \param _disp_noise_factor displacement noise factor
     * \param _rot_noise_factor rotation noise factor
     *
     **/
    SensorCamera(StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr, int _img_width, int _img_height);

    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~SensorCamera();

    int img_width_;
    int img_height_;

};

} // namespace wolf

//#include "yaml-cpp/yaml.h"

namespace wolf{

// Define the factory method and register it in the SensorFactory
namespace
{
SensorBase* createCamera(std::string& _name, std::string _params_filename = "")
{
    SensorBase* sen = new SensorCamera(nullptr, nullptr, nullptr,0,0);
    sen->setName(_name);
    return sen;
}
const bool registered_camera = SensorFactory::get()->registerSensor("CAMERA", createCamera);
}

} // namespace wolf

#endif // SENSOR_CAMERA_H
