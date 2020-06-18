#ifndef SENSOR_CAMERA_H
#define SENSOR_CAMERA_H

//wolf includes
#include "core/sensor/sensor_base.h"
#include "core/utils/params_server.h"

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ParamsSensorCamera);
/** Struct of intrinsic camera parameters
 */
struct ParamsSensorCamera : public ParamsSensorBase
{
        unsigned int width;                     ///< Image width in pixels
        unsigned int height;                    ///< Image height in pixels
        Eigen::Vector4d pinhole_model_raw;      ///< k = [u_0, v_0, alpha_u, alpha_v]  vector of pinhole intrinsic parameters
        Eigen::Vector4d pinhole_model_rectified;///< k = [u_0, v_0, alpha_u, alpha_v]  vector of pinhole intrinsic parameters
        Eigen::VectorXd distortion;             ///< d = [d_1, d_2, d_3, ...] radial distortion coefficients
    ParamsSensorCamera()
    {
        //DEFINED FOR COMPATIBILITY PURPOSES. TO BE REMOVED IN THE FUTURE.
    }
    ParamsSensorCamera(std::string _unique_name, const ParamsServer& _server):
        ParamsSensorBase(_unique_name,  _server)
    {
        width                   = _server.getParam<unsigned int>(prefix + _unique_name       + "/width");
        height                  = _server.getParam<unsigned int>(prefix + _unique_name       + "/height");
        pinhole_model_raw       = _server.getParam<Eigen::Vector4d>(prefix + _unique_name    + "/pinhole_model_raw");
        pinhole_model_rectified = _server.getParam<Eigen::Vector4d>(prefix + _unique_name    + "/pinhole_model_rectified");
        distortion              = _server.getParam<Eigen::VectorXd>(prefix + _unique_name    + "/distortion");
    }
    std::string print() const
    {
        return "\n" + ParamsSensorBase::print()                                               + "\n"
            + "width: "         + std::to_string(width)                                     + "\n"
            + "height: "        + std::to_string(height)                                    + "\n"
            + "pinhole: "       + converter<std::string>::convert(pinhole_model_raw)        + "\n"
            + "pinhole: "       + converter<std::string>::convert(pinhole_model_rectified)  + "\n"
            + "distortion: "    + converter<std::string>::convert(distortion)               + "\n";
    }
        ~ParamsSensorCamera() override = default;
};

WOLF_PTR_TYPEDEFS(SensorCamera);
/**Pin-hole camera sensor
 */
class SensorCamera : public SensorBase
{
    public:

        SensorCamera(const Eigen::VectorXd & _extrinsics, const ParamsSensorCamera& _intrinsics);
        SensorCamera(const Eigen::VectorXd & _extrinsics, ParamsSensorCameraPtr _intrinsics_ptr);
        WOLF_SENSOR_CREATE(SensorCamera, ParamsSensorCamera, 7);

        ~SensorCamera() override;

        Eigen::VectorXd getDistortionVector()   { return distortion_; }
        Eigen::VectorXd getCorrectionVector()   { return correction_; }
        Eigen::Matrix3d getIntrinsicMatrix()    { return K_; }

        bool isUsingRawImages() { return using_raw_; }
        bool useRawImages();
        bool useRectifiedImages();


        int getImgWidth(){return img_width_;}
        int getImgHeight(){return img_height_;}
        void setImgWidth(int _w){img_width_ = _w;}
        void setImgHeight(int _h){img_height_ = _h;}

    private:
        int img_width_;
        int img_height_;
        Eigen::VectorXd distortion_;
        Eigen::VectorXd correction_;
        Eigen::Vector4d pinhole_model_raw_, pinhole_model_rectified_;
        Eigen::Matrix3d K_;
        bool using_raw_;

        virtual Eigen::Matrix3d setIntrinsicMatrix(Eigen::Vector4d _pinhole_model);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // to guarantee alignment (see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html)
};

inline bool SensorCamera::useRawImages()
{
    getIntrinsic()->setState(pinhole_model_raw_);
    K_ = setIntrinsicMatrix(pinhole_model_raw_);
    using_raw_ = true;

    return true;
}

inline bool SensorCamera::useRectifiedImages()
{
    getIntrinsic()->setState(pinhole_model_rectified_);
    K_ = setIntrinsicMatrix(pinhole_model_rectified_);
    using_raw_ = false;

    return true;
}

} // namespace wolf

#endif // SENSOR_CAMERA_H
