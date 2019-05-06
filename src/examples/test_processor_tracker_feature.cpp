/**
 * \file test_processor_tracker_feature.cpp
 *
 *  Created on: Apr 11, 2016
 *      \author: jvallve
 */

//std
#include <iostream>

//Wolf
#include "base/common/wolf.h"
#include "base/problem/problem.h"
#include "base/sensor/sensor_base.h"
#include "base/state_block/state_block.h"
#include "base/processor/processor_tracker_feature_dummy.h"
#include "base/capture/capture_void.h"

void print_prc(wolf::ProcessorTrackerFeaturePtr _prc)
{
    using namespace wolf;

    auto o = _prc->getOriginPtr();
    auto l = _prc->getLastPtr();
    auto i = _prc->getIncomingPtr();

    std::cout <<   "o: C" << ( int)( o ? o->id() : -99 )
            << " || l: C" << ( int)( l ? l->id() : -99 )
            << " || i: C" << ( int)( i ? i->id() : -99 )
            << std::endl;
    for (auto ftr : _prc->getLastPtr()->getFeatureList())
    {
        SizeStd trk_id = ftr->trackId();
        std::cout << "Track " << trk_id << " ---------------------" << std::endl;
        for (auto ftr_pair : _prc->track(trk_id))
        {
            auto f = ftr_pair.second;
            auto C = f->getCapturePtr();
            std::cout << "f" << f->id() << " C" << (int)(C ? C->id() : -99) << std::endl;
        }
    }
}

int main()
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    std::cout << std::endl << "==================== processor tracker feature test ======================" << std::endl;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create("PO 2D");
    SensorBasePtr sensor_ptr_ = make_shared<SensorBase>("ODOM 2D", std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)),
                                             std::make_shared<StateBlock>(Eigen::VectorXs::Zero(1)),
                                             std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), 2);

    ProcessorParamsTrackerFeaturePtr params_trk = std::make_shared<ProcessorParamsTrackerFeature>();
    params_trk->voting_active = true;
    params_trk->max_new_features = 7;
    params_trk->min_features_for_keyframe = 4;
    params_trk->time_tolerance = 0.25;
    shared_ptr<ProcessorTrackerFeatureDummy> processor_ptr_ = make_shared<ProcessorTrackerFeatureDummy>(params_trk);

    wolf_problem_ptr_->addSensor(sensor_ptr_);
    sensor_ptr_->addProcessor(processor_ptr_);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    TimeStamp t(0);
    Scalar dt = 0.5;
    for (auto i = 0; i < 10; i++)
    {
        std::cout << "\n===== Capture TS = " << t << " =====" << std::endl;
        processor_ptr_->process(make_shared<CaptureVoid>(t, sensor_ptr_));

        print_prc(processor_ptr_);

        t += dt;
    }

    wolf_problem_ptr_->print(2);

    return 0;
}

