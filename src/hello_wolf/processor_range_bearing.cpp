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

ProcessorRangeBearing::ProcessorRangeBearing(const SensorRangeBearingPtr _sensor_ptr, ProcessorParamsBasePtr _params) :
        ProcessorBase("RANGE BEARING", _params)
{
    H_r_s   = transform(_sensor_ptr->getPPtr()->getState(), _sensor_ptr->getOPtr()->getState());
}

void ProcessorRangeBearing::process(CaptureBasePtr _capture)
{

    // 1. get KF
    FrameBasePtr kf(nullptr);
    if ( !kf_pack_buffer_.empty() )
    {
        // KeyFrame Callback received
        KFPackPtr pack = kf_pack_buffer_.selectPack( _capture->getTimeStamp(), params_->time_tolerance );

        if (pack!=nullptr)
            kf = pack->key_frame;

        kf_pack_buffer_.removeUpTo( _capture->getTimeStamp() );

        assert( kf && "Callback KF is not close enough to _capture!");
    }

    if (!kf)
    {
        // No KeyFrame callback received -- we assume a KF is available to hold this _capture (checked in assert below)
        kf = getProblem()->closestKeyFrameToTimeStamp(_capture->getTimeStamp());
        assert( (fabs(kf->getTimeStamp() - _capture->getTimeStamp()) < params_->time_tolerance) && "Could not find a KF close enough to _capture!");
    }

    // 2. cast incoming capture to the range-and-bearing type, add it to the keyframe
    CaptureRangeBearingPtr capture_rb = std::static_pointer_cast<CaptureRangeBearing>(_capture);
    kf->addCapture(capture_rb);

    // 3. explore all observations in the capture
    for (SizeEigen i = 0; i < capture_rb->getIds().size(); i++)
    {
        // extract info
        int     id      = capture_rb->getId     (i);
        Scalar  range   = capture_rb->getRange  (i);
        Scalar  bearing = capture_rb->getBearing(i);

        // 4. create or recover landmark
        LandmarkPoint2DPtr lmk;
        auto lmk_it = known_lmks.find(id);
        if ( lmk_it != known_lmks.end() )
        {
            // known landmarks : recover landmark
            lmk = std::static_pointer_cast<LandmarkPoint2D>(lmk_it->second);
            WOLF_TRACE("known lmk(", id, "): ", lmk->getPPtr()->getState().transpose());
        }
        else
        {
            // new landmark:
            // - create landmark
            lmk = std::make_shared<LandmarkPoint2D>(id, invObserve(range, bearing));
            WOLF_TRACE("new   lmk(", id, "): ", lmk->getPPtr()->getState().transpose());
            getProblem()->getMapPtr()->addLandmark(lmk);
            // - add to known landmarks
            known_lmks.emplace(id, lmk);
        }

        // 5. create feature
        Vector2s rb(range,bearing);
        auto ftr = std::make_shared<FeatureRangeBearing>(rb,
                                                         getSensorPtr()->getNoiseCov());
        capture_rb->addFeature(ftr);

        // 6. create constraint
        auto prc = shared_from_this();
        auto ctr = std::make_shared<ConstraintRangeBearing>(capture_rb,
                                                            lmk,
                                                            prc,
                                                            false,
                                                            CTR_ACTIVE);
        ftr->addConstraint(ctr);
        lmk->addConstrainedBy(ctr);
    }

}

ProcessorBasePtr ProcessorRangeBearing::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sen_ptr)
{
    SensorRangeBearingPtr       sensor_rb = std::static_pointer_cast<SensorRangeBearing>(_sen_ptr);
    ProcessorParamsRangeBearingPtr params = std::static_pointer_cast<ProcessorParamsRangeBearing>(_params);

    // construct processor
    ProcessorRangeBearingPtr prc = std::make_shared<ProcessorRangeBearing>(sensor_rb, params);

    // setup processor
    prc->setName(_unique_name);

    return prc;
}

Eigen::Vector2s ProcessorRangeBearing::observe(const Eigen::Vector2s& lmk_w) const
{
    return polar(toSensor(lmk_w));
}

Eigen::Vector2s ProcessorRangeBearing::invObserve(Scalar r, Scalar b) const
{
    return fromSensor(rect(r, b));
}

ProcessorRangeBearing::Trf ProcessorRangeBearing::transform(const Eigen::Vector3s& _pose) const
{
    Trf H = Eigen::Translation<Scalar,2>(_pose(0), _pose(1)) * Eigen::Rotation2D<Scalar>(_pose(2)) ;
    return H;
}

ProcessorRangeBearing::Trf ProcessorRangeBearing::transform(const Eigen::Vector2s& _position,
                                                            const Eigen::Vector1s& _orientation) const
{
    Trf H = Eigen::Translation<Scalar,2>(_position(0), _position(1)) * Eigen::Rotation2D<Scalar>(_orientation(0)) ;
    return H;
}

Eigen::Vector2s ProcessorRangeBearing::fromSensor(const Eigen::Vector2s& lmk_s) const
{
    Trf H_w_r = transform(getProblem()->getCurrentState());
    return H_w_r * H_r_s * lmk_s;
}

Eigen::Vector2s ProcessorRangeBearing::toSensor(const Eigen::Vector2s& lmk_w) const
{
//    Trf H_w_r = transform(getSensorPtr()->getPPtr()->getState(), getSensorPtr()->getOPtr()->getState());
    Trf H_w_r = transform(getProblem()->getCurrentState());
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

} /* namespace wolf */


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf
{
WOLF_REGISTER_PROCESSOR("RANGE BEARING", ProcessorRangeBearing)
} // namespace wolf

