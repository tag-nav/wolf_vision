/*
 * ProcessorRangeBearing.h
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#ifndef HELLO_WOLF_PROCESSOR_RANGE_BEARING_H_
#define HELLO_WOLF_PROCESSOR_RANGE_BEARING_H_

#include "processor_base.h"
#include "sensor_range_bearing.h"
#include "Eigen/Geometry"

#include <map>

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsRangeBearing);

struct ProcessorParamsRangeBearing : public ProcessorParamsBase
{
        //        Eigen::Vector3s pose0, delta;
};


using namespace Eigen;
WOLF_PTR_TYPEDEFS(ProcessorRangeBearing);

class ProcessorRangeBearing : public ProcessorBase
{
    public:
        typedef Eigen::Transform<Scalar, 2, Eigen::Affine> Trf;

        ProcessorRangeBearing(const SensorRangeBearingPtr _sensor_ptr, const Scalar& _time_tolerance = 0);
        virtual ~ProcessorRangeBearing() {/* empty */}

        // Factory method for high level API
        static ProcessorBasePtr create(const std::string& _unique_name,
                                       const ProcessorParamsBasePtr _params,
                                       const SensorBasePtr sensor_ptr = nullptr);

    protected:
        // Implementation of pure virtuals from ProcessorBase
        virtual void process            (CaptureBasePtr _capture) override;
        virtual bool voteForKeyFrame    () override {return false;}
        virtual bool keyFrameCallback   (FrameBasePtr _key_frame, const Scalar& _time_tolerance) override;

        // landmark observation models -- they would be better off in a separate library e.g. range_bearing_tools.h
        Eigen::Vector2s observe     (const Eigen::Vector2s& lmk_w) const;
        Eigen::Vector2s invObserve  (Scalar r, Scalar b) const;

    private:
        // control variables
        Trf H_r_s; // transformation matrix, robot to sensor
        std::map<int, LandmarkBasePtr> known_lmks; // all observed lmks so far

    private:
        // helper methods -- to be used only here -- they would be better off in a separate library e.g. range_bearing_tools.h
        Trf             transform   (const Eigen::Vector3s& _pose) const;
        Trf             transform   (const Eigen::Vector2s& _position, const Eigen::Vector1s& _orientation) const;
        Eigen::Vector2s fromSensor  (const Eigen::Vector2s& lmk_s) const;
        Eigen::Vector2s toSensor    (const Eigen::Vector2s& lmk_w) const;
        Eigen::Vector2s polar       (const Eigen::Vector2s& rect) const;
        Eigen::Vector2s rect        (Scalar range, Scalar bearing) const;
};

} /* namespace wolf */

#endif /* HELLO_WOLF_PROCESSOR_RANGE_BEARING_H_ */
