//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
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
        unsigned int    width;                  ///< Image width in pixels
        unsigned int    height;                 ///< Image height in pixels
        bool            using_raw;              ///< Use raw images?
        Eigen::Vector4d pinhole_model_raw;      ///< k = [u_0, v_0, alpha_u, alpha_v]  vector of pinhole intrinsic parameters
        Eigen::Vector4d pinhole_model_rectified;///< k = [u_0, v_0, alpha_u, alpha_v]  vector of pinhole intrinsic parameters
        Eigen::VectorXd distortion;             ///< d = [d_1, d_2, d_3, ...] radial distortion coefficients
    ParamsSensorCamera() :
        width(0),
        height(0),
        using_raw(true)
        {
            //
        };

    ParamsSensorCamera(std::string _unique_name, const ParamsServer& _server):
        ParamsSensorBase(_unique_name,  _server)
    {
        width               = _server.getParam<unsigned int>(prefix + _unique_name       + "/width");
        height              = _server.getParam<unsigned int>(prefix + _unique_name       + "/height");
        using_raw           = _server.getParam<bool>        (prefix + _unique_name       + "/using_raw");
        VectorXd distort    = _server.getParam<Eigen::VectorXd>(prefix + _unique_name + "/distortion_coefficients/data");
        VectorXd intrinsic  = _server.getParam<Eigen::VectorXd>(prefix + _unique_name + "/camera_matrix/data");
        VectorXd projection = _server.getParam<Eigen::VectorXd>(prefix + _unique_name + "/projection_matrix/data");

        /* Get raw params from K matrix in camera_info:
         *
         *     | au  0 u0 |
         * K = |  0 av v0 |
         *     |  0  0  1 |
         */
        pinhole_model_raw[0] = intrinsic[2]; // u0
        pinhole_model_raw[1] = intrinsic[5]; // v0
        pinhole_model_raw[2] = intrinsic[0]; // au
        pinhole_model_raw[3] = intrinsic[4]; // av

        /* Get rectified params from P matrix in camera_info:
         *
         *     | au  0 u0 tx |
         * P = |  0 av v0 ty |
         *     |  0  0  1  0 |
         */
        pinhole_model_rectified[0] = projection[2]; // u0
        pinhole_model_rectified[1] = projection[6]; // v0
        pinhole_model_rectified[2] = projection[0]; // au
        pinhole_model_rectified[3] = projection[5]; // av

        /* Get distortion params from vector D in camera_info:
         *
         * D = [ r1 r2 t1 t2 r3 ], with ri: dadial params; ti: tangential params
         *
         * NOTE: Wolf ignores tangential params!!!
         */
        assert (distort.size() == 5 && "Distortion size must be size 5!");

        WOLF_WARN_COND( distort(2) != 0 || distort(3) != 0 , "Wolf does not handle tangential distortion. Please consider re-calibrating without tangential distortion!");

        if (distort(4) == 0)
            if (distort(1) == 0)
                if (distort(0) == 0)
                    distortion.resize(0);
                else
                {
                    distortion.resize(1);
                    distortion = distort.head<1>();
                }
            else
            {
                distortion.resize(2);
                distortion = distort.head<2>();
            }
        else
        {
            distortion.resize(3);
            distortion.head<2>() = distort.head<2>();
            distortion.tail<1>() = distort.tail<1>();
        }
    }
    std::string print() const override
    {
        return ParamsSensorBase::print()                                                    + "\n"
            + "width: "         + std::to_string(width)                                     + "\n"
            + "height: "        + std::to_string(height)                                    + "\n"
            + "using_raw: "     + std::to_string(using_raw)                                 + "\n"
            + "pinhole: "       + converter<std::string>::convert(pinhole_model_raw)        + "\n"
            + "pinhole rect.: " + converter<std::string>::convert(pinhole_model_rectified)  + "\n"
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
        Eigen::Vector4d getPinholeModel()       { if (using_raw_) return pinhole_model_raw_; else return pinhole_model_rectified_;}

        bool isUsingRawImages() { return using_raw_; }
        bool useRawImages();
        bool useRectifiedImages();

        int getImgWidth()           {return img_width_;}
        int getImgHeight()          {return img_height_;}
        void setImgWidth(int _w)    {img_width_ = _w;}
        void setImgHeight(int _h)   {img_height_ = _h;}

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
    distortion_.resize(0); // remove distortion from model
    correction_.resize(0);
    using_raw_ = false;

    return true;
}

} // namespace wolf

#endif // SENSOR_CAMERA_H
