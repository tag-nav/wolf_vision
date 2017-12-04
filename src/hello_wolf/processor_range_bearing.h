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

#include <set>

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsRangeBearing);

struct ProcessorParamsRangeBearing : public ProcessorParamsBase
{
        Eigen::Vector3s pose0, delta;
};


using namespace Eigen;
WOLF_PTR_TYPEDEFS(ProcessorRangeBearing);

class ProcessorRangeBearing : public ProcessorBase
{
    public:
        typedef Eigen::Transform<Scalar, 2, Eigen::Affine> Trf;

        ProcessorRangeBearing(const SensorRangeBearingPtr _sensor_ptr, const Vector3s& _pose0, const Vector3s& _delta, const Scalar& _time_tolerance = 0);
        virtual ~ProcessorRangeBearing();

        // Implementation of pure virtuals from ProcessorBase
        virtual void process(CaptureBasePtr _capture) override;
        virtual bool voteForKeyFrame() override {return false;}
        virtual bool keyFrameCallback(FrameBasePtr _key_frame, const Scalar& _time_tolerance) override {return true;}

        // helpers
        Eigen::Vector2s observe(const Eigen::Vector2s& lmk_w) const;
        Eigen::Vector2s invObserve(Scalar r, Scalar b) const;

        // Factory method for high level API
        static ProcessorBasePtr create(const std::string& _unique_name,
                                       const ProcessorParamsBasePtr _params,
                                       const SensorBasePtr sensor_ptr = nullptr);

    private:
        // control variables
        Size step_number;
        Trf H_w_r, H_r_s, H_delta; // transformation matrices, world to robot, to sensor, and motion delta
        std::set<int> observed_ids; // ids of all observed lmks so far

        // helper methods
        Trf transform(const Eigen::Vector3s& _pose);
        Trf transform(const Eigen::Vector2s& _position, const Eigen::Vector1s& _orientation);
        Eigen::Vector2s fromSensor(const Eigen::Vector2s& lmk_s) const;
        Eigen::Vector2s   toSensor(const Eigen::Vector2s& lmk_w) const;
        Eigen::Vector2s polar(const Eigen::Vector2s& rect) const;
        Eigen::Vector2s rect(Scalar range, Scalar bearing) const;
};

} /* namespace wolf */

#endif /* HELLO_WOLF_PROCESSOR_RANGE_BEARING_H_ */
