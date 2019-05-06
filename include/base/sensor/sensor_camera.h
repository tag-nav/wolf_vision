#ifndef SENSOR_CAMERA_H
#define SENSOR_CAMERA_H

//wolf includes
#include "base/sensor/sensor_base.h"

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(IntrinsicsCamera);
/** Struct of intrinsic camera parameters
 */
struct IntrinsicsCamera : public IntrinsicsBase
{
        unsigned int width;                     ///< Image width in pixels
        unsigned int height;                    ///< Image height in pixels
        Eigen::Vector4s pinhole_model_raw;      ///< k = [u_0, v_0, alpha_u, alpha_v]  vector of pinhole intrinsic parameters
        Eigen::Vector4s pinhole_model_rectified;///< k = [u_0, v_0, alpha_u, alpha_v]  vector of pinhole intrinsic parameters
        Eigen::VectorXs distortion;             ///< d = [d_1, d_2, d_3, ...] radial distortion coefficients

        virtual ~IntrinsicsCamera() = default;
};

WOLF_PTR_TYPEDEFS(SensorCamera);
/**Pin-hole camera sensor
 */
class SensorCamera : public SensorBase
{
    public:

        SensorCamera(const Eigen::VectorXs & _extrinsics, const IntrinsicsCamera& _intrinsics);
        SensorCamera(const Eigen::VectorXs & _extrinsics, IntrinsicsCameraPtr _intrinsics_ptr);

        virtual ~SensorCamera();

        Eigen::VectorXs getDistortionVector()   { return distortion_; }
        Eigen::VectorXs getCorrectionVector()   { return correction_; }
        Eigen::Matrix3s getIntrinsicMatrix()    { return K_; }

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
        Eigen::VectorXs distortion_;
        Eigen::VectorXs correction_;
        Eigen::Vector4s pinhole_model_raw_, pinhole_model_rectified_;
        Eigen::Matrix3s K_;
        bool using_raw_;

        virtual Eigen::Matrix3s setIntrinsicMatrix(Eigen::Vector4s _pinhole_model);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // to guarantee alignment (see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html)

        static SensorBasePtr create(const std::string & _unique_name, //
                                    const Eigen::VectorXs& _extrinsics, //
                                    const IntrinsicsBasePtr _intrinsics);

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
