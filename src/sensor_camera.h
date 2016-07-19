#ifndef SENSOR_CAMERA_H
#define SENSOR_CAMERA_H

//wolf includes
#include "sensor_base.h"

namespace wolf
{

/** Struct of intrinsic camera parameters
 */
struct IntrinsicsCamera : public IntrinsicsBase
{
        unsigned int width;                 ///< Image width in pixels
        unsigned int height;                ///< Image height in pixels
        Eigen::Vector4s pinhole_model;      ///< k = [u_0, v_0, alpha_u, alpha_v]  vector of pinhole intrinsic parameters
        Eigen::VectorXs distortion;         ///< d = [d_1, d_2, d_3, ...] radial distortion coefficients
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

        SensorCamera(const Eigen::VectorXs & _extrinsics, const IntrinsicsCamera * _intrinsics_ptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~SensorCamera();

        Eigen::VectorXs getDistortionVector(){return distortion_;}
        Eigen::VectorXs getCorrectionVector(){return correction_;}
        int getImgWidth(){return img_width_;}
        int getImgHeight(){return img_height_;}

    private:
        int img_width_;
        int img_height_;
        Eigen::VectorXs distortion_;
        Eigen::VectorXs correction_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // to guarantee alignment (see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html)

        static SensorBase* create(const std::string & _unique_name, //
                                  const Eigen::VectorXs& _extrinsics, //
                                  const IntrinsicsBase* _intrinsics);

};

} // namespace wolf

#endif // SENSOR_CAMERA_H
