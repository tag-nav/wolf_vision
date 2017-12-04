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
#include "constraint_range_bearing.h"

namespace wolf
{

ProcessorRangeBearing::ProcessorRangeBearing(const SensorRangeBearingPtr _sensor_ptr, const Scalar& _time_tolerance) :
        ProcessorBase("RANGE BEARING", _time_tolerance)
{
    H_r_s   = transform(_sensor_ptr->getPPtr()->getState(), _sensor_ptr->getOPtr()->getState());
}

void ProcessorRangeBearing::process(CaptureBasePtr _capture)
{
    CaptureRangeBearingPtr capture = std::static_pointer_cast<CaptureRangeBearing>(_capture);

    // 1. create KF
    VectorXs    x(3);
    TimeStamp   ts;
    getProblem()->getCurrentStateAndStamp(x, ts);
    FrameBasePtr kf = getProblem()->emplaceFrame(KEY_FRAME, x, ts);
    getProblem()->keyFrameCallback(kf, shared_from_this(), 0.1);

    // 2. create Capture
    CaptureRangeBearingPtr cap = std::make_shared<CaptureRangeBearing>(capture->getTimeStamp(),
                                                                       getSensorPtr(),
                                                                       capture->getIds(),
                                                                       capture->getRanges(),
                                                                       capture->getBearings());

    // 3. explore all observations in the capture
    for (Size i = 0; i < capture->getIds().size(); i++)
    {
        // extract info
        int     id      = capture->getId     (i);
        Scalar  range   = capture->getRange  (i);
        Scalar  bearing = capture->getBearing(i);
        WOLF_TRACE("id(", i, ") = ", id, "; range(", i, ") = ", range, "; bearing(", i, ") = ", bearing);

        // 4. create or recover landmark
        LandmarkPoint2DPtr lmk;
        auto it = lmk_ids.find(id);
        if ( it == lmk_ids.end() )
        {
            // new landmark:
            // - create landmark
            lmk = std::make_shared<LandmarkPoint2D>(id, invObserve(range, bearing));
            getProblem()->getMapPtr()->addLandmark(lmk);
            // - add to known landmarks
            lmk_ids.emplace(id, lmk);
        }
        else
        {
            // known landmarks : recover landmark
            lmk = std::static_pointer_cast<LandmarkPoint2D>(it->second);
        }

        // 5. create feature
        Vector2s rb(range,bearing);
        FeatureRangeBearingPtr ftr = std::make_shared<FeatureRangeBearing>(rb,
                                                                           getSensorPtr()->getNoiseCov());
        cap->addFeature(ftr);

        // 6. create constraint
        ConstraintRangeBearingPtr ctr = std::make_shared<ConstraintRangeBearing>(lmk,
                                                                                 shared_from_this(),
                                                                                 false,
                                                                                 CTR_ACTIVE);
        ftr->addConstraint(ctr);
        lmk->addConstrainedBy(ctr);
    }

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
    ProcessorRangeBearingPtr prc = std::make_shared<ProcessorRangeBearing>(sensor_rb, params->time_tolerance);

    // setup processor
    prc->setName(_unique_name);

    return prc;
}

ProcessorRangeBearing::Trf ProcessorRangeBearing::transform(const Eigen::Vector3s& _pose) const
{
    Trf H = Eigen::Translation<Scalar,2>(_pose(0), _pose(1)) * Eigen::Rotation2D<Scalar>(_pose(2)) ;
    return H;
}

Eigen::Vector2s ProcessorRangeBearing::fromSensor(const Eigen::Vector2s& lmk_s) const
{
    Eigen::Vector2s pos_s = getSensorPtr()->getPPtr()->getState();
    Eigen::Vector1s ori_s = getSensorPtr()->getOPtr()->getState();
    Trf H_w_r = transform(pos_s, ori_s);
    return H_w_r * H_r_s * lmk_s;
}

Eigen::Vector2s ProcessorRangeBearing::toSensor(const Eigen::Vector2s& lmk_w) const
{
    Trf H_w_r = transform(getSensorPtr()->getPPtr()->getState(), getSensorPtr()->getOPtr()->getState());
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
                                                            const Eigen::Vector1s& _orientation) const
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

