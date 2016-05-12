#include "sensor_camera.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "yaml-cpp/yaml.h"
#include "pinholeTools.h"

namespace wolf {

SensorCamera::SensorCamera(StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr,
                           int _img_width, int _img_height) :
        SensorBase(SEN_CAMERA, _p_ptr, _o_ptr, _intr_ptr, 2),
        img_width_(_img_width),
        img_height_(_img_height)
{
    setType("CAMERA");
}

SensorCamera::~SensorCamera()
{
    //
}

// Define the factory method
SensorBase* SensorCamera::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq,
                                 const IntrinsicsBase* _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");
    StateBlock* pos_ptr = new StateBlock(_extrinsics_pq.head(3), true);
    StateBlock* ori_ptr = new StateQuaternion(_extrinsics_pq.tail(4), true);
    // cast instrinsics to good type and extract intrinsic vector
    IntrinsicsCamera* intrinsics = (IntrinsicsCamera*)((_intrinsics));
    StateBlock* intr_ptr = new StateBlock(intrinsics->intrinsic_vector);
    // Construct camera and give it a name
    SensorCamera* sen_ptr = new SensorCamera(pos_ptr, ori_ptr, intr_ptr, intrinsics->width, intrinsics->height);
    sen_ptr->setName(_unique_name);
    return sen_ptr;
}

IntrinsicsBase* SensorCamera::createIntrinsics(const std::string & _filename_dot_yaml)
{
    YAML::Node camera_config = YAML::LoadFile(_filename_dot_yaml);

    if (camera_config["sensor type"])
    {

        // YAML:: to std::
        std::string sensor_type = camera_config["sensor type"].as<std::string>();
        std::string sensor_name = camera_config["sensor name"].as<std::string>();
        std::vector<double> p   = camera_config["extrinsic"] ["position"].as<std::vector<double> >(); // in one go: it works!
        std::vector<double> o   = camera_config["extrinsic"] ["orientation"].as<std::vector<double> >();
        std::vector<double> s   = camera_config["parameters"]["image size"].as<std::vector<double> >();
        std::vector<double> k   = camera_config["parameters"]["intrinsic"].as<std::vector<double> >();
        std::vector<double> d   = camera_config["parameters"]["distortion"].as<std::vector<double> >();
        std::vector<double> c   = camera_config["parameters"]["correction"].as<std::vector<double> >();

        // std:: to Eigen::
        // Using Eigen vector constructors from data pointers. Mind the vector sizes!
        using namespace Eigen;
        Vector3d pos(p.data());
        Vector3d ori(o.data());
        ori *= M_PI / 180; // deg to rad
        Vector2d size(s.data());
        Vector4d intrinsic(k.data());
        Map<VectorXd> distortion(d.data(), d.size());
        VectorXs correction(d.size());

        pinhole::computeCorrectionModel(intrinsic, distortion, correction);

        std::cout << "sensor type: " << sensor_type << std::endl;
        std::cout << "sensor name: " << sensor_name << std::endl;
        std::cout << "sensor extrinsics: " << std::endl;
        std::cout << "\tposition    : " << pos.transpose() << std::endl;
        std::cout << "\torientation : " << ori.transpose() << std::endl;
        std::cout << "sensor parameters: " << std::endl;
        std::cout << "\timage size  : " << size.transpose() << std::endl;
        std::cout << "\tintrinsic   : " << intrinsic.transpose() << std::endl;
        std::cout << "\tdistoriton  : " << distortion.transpose() << std::endl;
        std::cout << "\tcorrection  : " << correction.transpose() << std::endl;

        // Eigen:: to wolf::
        IntrinsicsCamera* intrinsics_cam = new IntrinsicsCamera;
        intrinsics_cam->type = sensor_type;
        intrinsics_cam->name = sensor_name;
        intrinsics_cam->intrinsic_vector = intrinsic;
        intrinsics_cam->distortion = distortion;
        intrinsics_cam->correction = correction;
        intrinsics_cam->width = size(0);
        intrinsics_cam->height = size(1);

        return intrinsics_cam;
    }

    std::cout << "Bad configuration file. No sensor type found." << std::endl;
    return nullptr;
}


} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
#include "intrinsics_factory.h"
namespace wolf {
namespace
{
const bool registered_camera_intr = IntrinsicsFactory::get()->registerCreator("CAMERA", SensorCamera::createIntrinsics);
const bool registered_camera = SensorFactory::get()->registerCreator("CAMERA", SensorCamera::create);
}
} // namespace wolf

