/*
 * processor_range_bearing.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#include "processor_range_bearing.h"
#include "capture_range_bearing.h"
#include "landmark_point_2D.h"
#include "feature_range_bearing.h"

namespace wolf
{

ProcessorRangeBearing::ProcessorRangeBearing(const SensorRangeBearingPtr _sensor_ptr, const Vector3s& _pose0, const Vector3s& _delta, const Scalar& _time_tolerance) :
        ProcessorBase("RANGE BEARING", _time_tolerance),
        step_number(0)
{
    H_w_r   = transform(_pose0);
    H_r_s   = transform(getSensorPtr()->getPPtr()->getState(), getSensorPtr()->getOPtr()->getState());
    H_delta = transform(_delta);
}

ProcessorRangeBearing::~ProcessorRangeBearing()
{
    //
}

void ProcessorRangeBearing::process(CaptureBasePtr _capture)
{
    CaptureRangeBearingPtr capture = std::static_pointer_cast<CaptureRangeBearing>(_capture);

    // explore all observations in the capture
    for (Size i = 0; i < capture->getIds().size(); i++)
    {
        int     id      = capture->getId(i);
        Scalar  range   = capture->getRange  (i);
        Scalar  bearing = capture->getBearing(i);
        WOLF_TRACE("id(", i, ") = ", id, "; range(", i, ") = ", range, "; bearing(", i, ") = ", bearing);

        // create KF

        // create Capture

        // create or recover landmark

        LandmarkPoint2DPtr lmk;
        if (lmk_ids.find(id) == lmk_ids.end())
        {
            // new landmarks:
            // - create landmark
            lmk = std::make_shared<LandmarkPoint2D>(id, invObserve(range, bearing));
            getProblem()->getMapPtr()->addLandmark(lmk);
            // - add to known landmarks
            lmk_ids.insert(id);
        }
        else
        {
            // known landmarks : recover landmark
        }
        // create feature
        Vector2s rb(range,bearing);
        FeatureRangeBearingPtr ftr = std::make_shared<FeatureRangeBearing>(rb, getSensorPtr()->getNoiseCov());
        // create constraint
    }

    // advance one step
    H_w_r = H_w_r * H_delta;
    step_number ++ ;
}

Eigen::Vector2s ProcessorRangeBearing::observe(const Eigen::Vector2s& lmk_w) const
{
    return polar(toSensor(lmk_w));
}

Eigen::Vector2s ProcessorRangeBearing::invObserve(Scalar r, Scalar b) const
{
    return fromSensor(rect(r, b));
}

ProcessorBasePtr ProcessorRangeBearing::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sen_ptr)
{
    SensorRangeBearingPtr       sensor_rb = std::static_pointer_cast<SensorRangeBearing>(_sen_ptr);
    ProcessorParamsRangeBearingPtr params = std::static_pointer_cast<ProcessorParamsRangeBearing>(_params);

    // construct processor
    ProcessorRangeBearingPtr prc = std::make_shared<ProcessorRangeBearing>(sensor_rb, params->pose0, params->delta, params->time_tolerance);

    // setup processor
    prc->setName(_unique_name);

    return prc;
}

ProcessorRangeBearing::Trf ProcessorRangeBearing::transform(const Eigen::Vector3s& _pose)
{
    Trf H = Eigen::Translation<Scalar,2>(_pose(0), _pose(1)) * Eigen::Rotation2D<Scalar>(_pose(2)) ;
    return H;
}

Eigen::Vector2s ProcessorRangeBearing::fromSensor(const Eigen::Vector2s& lmk_s) const
{
    return H_w_r * H_r_s * lmk_s;
}

Eigen::Vector2s ProcessorRangeBearing::toSensor(const Eigen::Vector2s& lmk_w) const
{
    return (H_w_r * H_r_s).inverse() * lmk_w;
}

Eigen::Vector2s ProcessorRangeBearing::polar(const Eigen::Vector2s& rect) const
{
    Eigen::Vector2s polar;
    polar(0) = rect.norm();
    polar(1) = atan2(rect(1), rect(0));
    return polar;
}

Eigen::Vector2s ProcessorRangeBearing::rect(Scalar range, Scalar bearing) const
{
    return range * (Vector2s() << cos(bearing), sin(bearing)).finished();
}

ProcessorRangeBearing::Trf ProcessorRangeBearing::transform(const Eigen::Vector2s& _position,
                                                                                 const Eigen::Vector1s& _orientation)
{
    Trf H = Eigen::Translation<Scalar,2>(_position(0), _position(1)) * Eigen::Rotation2D<Scalar>(_orientation(0)) ;
    return H;
}

} /* namespace wolf */


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf
{
WOLF_REGISTER_PROCESSOR("RANGE BEARING", ProcessorRangeBearing)
} // namespace wolf

