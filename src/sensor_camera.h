#ifndef SENSOR_CAMERA_H
#define SENSOR_CAMERA_H

//wolf includes
#include "sensor_base.h"

namespace wolf {

/** Struct of all intrinsic camera parameters
 */
struct IntrinsicsCamera : public IntrinsicsBase
{
        unsigned int width;                 // Image width in pixels
        unsigned int height;                // Image height in pixels
        Eigen::Vector4s intrinsic_vector;   // k = [u_0, v_0, alpha_u, alpha_v]  vector of pinhole intrinsic parameters
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
     * \param _img_width image height in pixels
     * \param _img_height image width in pixels
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

    public:
        static SensorBase* create(const std::string & _name, const Eigen::VectorXs& _extrinsics, const IntrinsicsBase* _intrinsics);

};


} // namespace wolf

#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

// Define the factory method

inline SensorBase* SensorCamera::create(const std::string & _name, const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsBase* _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");
    StateBlock* pos_ptr = new StateBlock(_extrinsics_pq.head(3), true);
    StateBlock* ori_ptr = new StateQuaternion(_extrinsics_pq.tail(4), true);
    // cast instrinsics to good type and extract intrinsic vector
    IntrinsicsCamera* intrinsics = (IntrinsicsCamera*)(_intrinsics);
    StateBlock* intr_ptr = new StateBlock(intrinsics->intrinsic_vector);
    // Construct camera and give it a name
    SensorCamera* sen_ptr = new SensorCamera(pos_ptr, ori_ptr, intr_ptr, intrinsics->width, intrinsics->height);
    sen_ptr->setName(_name);
    return sen_ptr;
}

} // namespace wolf



// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
namespace
{
const bool registered_camera = SensorFactory::get()->registerCreator("CAMERA", SensorCamera::create);
}
} // namespace wolf

#endif // SENSOR_CAMERA_H
