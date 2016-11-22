#include "problem.h"
#include "constraint_base.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "hardware_base.h"
#include "trajectory_base.h"
#include "map_base.h"
#include "processor_motion.h"
#include "processor_tracker.h"
#include "sensor_base.h"
#include "sensor_gps.h"
#include "capture_fix.h"
#include "factory.h"
#include "sensor_factory.h"
#include "processor_factory.h"
#include "frame_imu.h"

namespace wolf
{

// unnamed namespace used for helper functions local to this file.
namespace
{
std::string uppercase(std::string s) {for (auto & c: s) c = std::toupper(c); return s;}
}


Problem::Problem(FrameStructure _frame_structure) :
        hardware_ptr_(std::make_shared<HardwareBase>()),
        trajectory_ptr_(std::make_shared<TrajectoryBase>(_frame_structure)),
        map_ptr_(std::make_shared<MapBase>()),
        processor_motion_ptr_(),
        origin_is_set_(false)
{
    //
}

void Problem::setup()
{
    hardware_ptr_->setProblem(shared_from_this());
    trajectory_ptr_->setProblem(shared_from_this());
    map_ptr_->setProblem(shared_from_this());
}

ProblemPtr Problem::create(FrameStructure _frame_structure)
{
    ProblemPtr p(new Problem(_frame_structure)); // We use `new` and not `make_shared` since the Problem constructor is private and cannot be passes to `make_shared`.
    p->setup();
    return p->shared_from_this();
}

Problem::~Problem()
{
//    WOLF_DEBUG("destructed -P");
}

void Problem::addSensor(SensorBasePtr _sen_ptr)
{
    getHardwarePtr()->addSensor(_sen_ptr);
}

SensorBasePtr Problem::installSensor(const std::string& _sen_type, //
                                   const std::string& _unique_sensor_name, //
                                   const Eigen::VectorXs& _extrinsics, //
                                   IntrinsicsBasePtr _intrinsics)
{
    SensorBasePtr sen_ptr = SensorFactory::get().create(uppercase(_sen_type), _unique_sensor_name, _extrinsics, _intrinsics);
    addSensor(sen_ptr);
    return sen_ptr;
}

SensorBasePtr Problem::installSensor(const std::string& _sen_type, //
                                   const std::string& _unique_sensor_name, //
                                   const Eigen::VectorXs& _extrinsics, //
                                   const std::string& _intrinsics_filename)
{
    if (_intrinsics_filename != "")
    {
        IntrinsicsBasePtr intr_ptr = IntrinsicsFactory::get().create(_sen_type, _intrinsics_filename);
        return installSensor(_sen_type, _unique_sensor_name, _extrinsics, intr_ptr);
    }
    else
        return installSensor(_sen_type, _unique_sensor_name, _extrinsics, std::shared_ptr<IntrinsicsBase>());

}

ProcessorBasePtr Problem::installProcessor(const std::string& _prc_type, //
                                         const std::string& _unique_processor_name, //
                                         SensorBasePtr _corresponding_sensor_ptr, //
                                         ProcessorParamsBasePtr _prc_params)
{
    ProcessorBasePtr prc_ptr = ProcessorFactory::get().create(uppercase(_prc_type), _unique_processor_name, _prc_params, _corresponding_sensor_ptr);
    _corresponding_sensor_ptr->addProcessor(prc_ptr);

    // setting the origin in all processor motion if origin already setted
    if (prc_ptr->isMotion() && origin_is_set_)
        (std::static_pointer_cast<ProcessorMotion>(prc_ptr))->setOrigin(getLastKeyFramePtr());

    // setting the main processor motion
    if (prc_ptr->isMotion() && processor_motion_ptr_ == nullptr)
        processor_motion_ptr_ = std::static_pointer_cast<ProcessorMotion>(prc_ptr);

    return prc_ptr;
}

ProcessorBasePtr Problem::installProcessor(const std::string& _prc_type, //
                               const std::string& _unique_processor_name, //
                               const std::string& _corresponding_sensor_name, //
                               const std::string& _params_filename)
{
    SensorBasePtr sen_ptr = getSensorPtr(_corresponding_sensor_name);
    if (sen_ptr == nullptr)
        throw std::runtime_error("Sensor not found. Cannot bind Processor.");
    if (_params_filename == "")
        return installProcessor(_prc_type, _unique_processor_name, sen_ptr, nullptr);
    else
    {
        ProcessorParamsBasePtr prc_params = ProcessorParamsFactory::get().create(_prc_type, _params_filename);
        return installProcessor(_prc_type, _unique_processor_name, sen_ptr, prc_params);
    }
}

wolf::SensorBasePtr Problem::getSensorPtr(const std::string& _sensor_name)
{
    auto sen_it = std::find_if(getHardwarePtr()->getSensorList().begin(),
                               getHardwarePtr()->getSensorList().end(),
                               [&](SensorBasePtr sb)
                               {
                                   return sb->getName() == _sensor_name;
                               }); // lambda function for the find_if

    if (sen_it == getHardwarePtr()->getSensorList().end())
        return nullptr;

    return (*sen_it);
}

ProcessorMotion::Ptr Problem::setProcessorMotion(const std::string& _processor_name)
{
    for (auto sen : getHardwarePtr()->getSensorList()) // loop all sensors
    {
        auto prc_it = std::find_if(sen->getProcessorList().begin(), // find processor by its name
                                   sen->getProcessorList().end(),
                                   [&](ProcessorBasePtr prc)
                                   {
                                       return prc->getName() == _processor_name;
                                   }); // lambda function for the find_if

        if (prc_it != sen->getProcessorList().end())  // found something!
        {
            if ((*prc_it)->isMotion()) // found, and it's motion!
            {
                std::cout << "Found processor '" << _processor_name << "', of type Motion, and set as the main motion processor." << std::endl;
                processor_motion_ptr_ = std::static_pointer_cast<ProcessorMotion>(*prc_it);
                return processor_motion_ptr_;
            }
            else // found, but it's not motion!
            {
                std::cout << "Found processor '" << _processor_name << "', but not of type Motion!" << std::endl;
                return nullptr;
            }
        }
    }
    // nothing found!
    std::cout << "Processor '" << _processor_name << "' not found!" << std::endl;
    return nullptr;
}


void Problem::setProcessorMotion(ProcessorMotion::Ptr _processor_motion_ptr)
{
    processor_motion_ptr_ = _processor_motion_ptr;
}

FrameBasePtr Problem::createFrame(FrameType _frame_type, const TimeStamp& _time_stamp)
{
    return createFrame(_frame_type, getStateAtTimeStamp(_time_stamp), _time_stamp);
}

FrameBasePtr Problem::createFrame(FrameType _frame_key_type, const Eigen::VectorXs& _frame_state,
                                const TimeStamp& _time_stamp)
{
    //std::cout << "Problem::createFrame" << std::endl;
    // ---------------------- CREATE NEW FRAME ---------------------
    // Create frame
    switch (trajectory_ptr_->getFrameStructure())
    {
        case FRM_PO_2D:
        {
            assert(_frame_state.size() == 3 && "Wrong state vector size");
            return trajectory_ptr_->addFrame(std::make_shared<FrameBase>(_frame_key_type, _time_stamp, std::make_shared<StateBlock>(_frame_state.head(2)),
                                  std::make_shared<StateBlock>(_frame_state.tail(1))));
        }
        case FRM_PO_3D:
        {
            assert(_frame_state.size() == 7 && "Wrong state vector size");
            return trajectory_ptr_->addFrame(std::make_shared<FrameBase>(_frame_key_type, _time_stamp, std::make_shared<StateBlock>(_frame_state.head(3)),
                                  std::make_shared<StateQuaternion>(_frame_state.tail(4))));
        }
        case FRM_POV_3D:
        {
            assert(_frame_state.size() == 10 && "Wrong state vector size");
            return trajectory_ptr_->addFrame(std::make_shared<FrameBase>(_frame_key_type, _time_stamp, std::make_shared<StateBlock>(_frame_state.head(3)),
                                  std::make_shared<StateQuaternion>(_frame_state.segment<4>(3)),
                                  std::make_shared<StateBlock>(_frame_state.tail(3))));
        }
        case FRM_PQVBB_3D:
        {
            assert(_frame_state.size() == 16 && "Wrong state vector size");
            return trajectory_ptr_->addFrame(std::make_shared<FrameIMU>(_frame_key_type, _time_stamp, _frame_state));
        }
        default:
            throw std::runtime_error(
                    "Unknown frame structure. Add appropriate frame structure to the switch statement.");
    }
}

Eigen::VectorXs Problem::getCurrentState()
{
    Eigen::VectorXs state(getFrameStructureSize());
    getCurrentState(state);
    return state;
}

Eigen::VectorXs Problem::getCurrentState(TimeStamp& ts)
{
    Eigen::VectorXs state(getFrameStructureSize());
    getCurrentState(state, ts);
    return state;
}

void Problem::getCurrentState(Eigen::VectorXs& state)
{
    TimeStamp ts;
    getCurrentState(state, ts);
}


void Problem::getCurrentState(Eigen::VectorXs& state, TimeStamp& ts)
{
    assert(state.size() == getFrameStructureSize() && "Problem::getCurrentState: bad state size");

    if (processor_motion_ptr_ != nullptr)
        processor_motion_ptr_->getCurrentState(state, ts);
    else if (trajectory_ptr_->getLastKeyFramePtr() != nullptr)
    {
        trajectory_ptr_->getLastKeyFramePtr()->getTimeStamp(ts);
        trajectory_ptr_->getLastKeyFramePtr()->getState(state);
    }
    else
        state = zeroState();
}

void Problem::getStateAtTimeStamp(const TimeStamp& _ts, Eigen::VectorXs& state)
{
    assert(state.size() == getFrameStructureSize() && "Problem::getStateAtTimeStamp: bad state size");

    if (processor_motion_ptr_ != nullptr)
    {
        processor_motion_ptr_->getState(_ts, state);
    }
    else
    {
        FrameBasePtr closest_frame = trajectory_ptr_->closestKeyFrameToTimeStamp(_ts);
        if (closest_frame != nullptr)
        {
            closest_frame->getState(state);
        }else{
            state = zeroState();
        }
    }
}

Eigen::VectorXs Problem::getStateAtTimeStamp(const TimeStamp& _ts)
{
    Eigen::VectorXs state(getFrameStructureSize());
    getStateAtTimeStamp(_ts, state);
    return state;
}

unsigned int Problem::getFrameStructureSize()
{
    switch (trajectory_ptr_->getFrameStructure())
    {
        case FRM_PO_2D:
            return 3;
        case FRM_PO_3D:
            return 7;
        case FRM_POV_3D:
            return 10;
        case FRM_PQVBB_3D:
            return 16;
        default:
            throw std::runtime_error(
                    "Problem::getFrameStructureSize(): Unknown frame structure. Add appropriate frame structure to the switch statement.");
    }
}

Eigen::VectorXs Problem::zeroState()
{
    Eigen::VectorXs state = Eigen::VectorXs::Zero(getFrameStructureSize());

    // Set the quaternion identity for 3D states. Set only the real part to 1:
    switch (trajectory_ptr_->getFrameStructure())
    {
        case FRM_PO_3D:
        case FRM_POV_3D:
        case FRM_PQVBB_3D:
            state(6) = 1.0;
            break;
        default:
            break;
    }

    return state;
}

bool Problem::permitKeyFrame(ProcessorBasePtr _processor_ptr)
{
    return true;
}

void Problem::keyFrameCallback(FrameBasePtr _keyframe_ptr, ProcessorBasePtr _processor_ptr, const Scalar& _time_tolerance)
{
    //std::cout << "Problem::keyFrameCallback: processor " << _processor_ptr->getName() << std::endl;
    for (auto sensor : hardware_ptr_->getSensorList())
    	for (auto processor : sensor->getProcessorList())
    		if (processor->id() != _processor_ptr->id())
                processor->keyFrameCallback(_keyframe_ptr, _time_tolerance);
}

LandmarkBasePtr Problem::addLandmark(LandmarkBasePtr _lmk_ptr)
{
    getMapPtr()->addLandmark(_lmk_ptr);
    return _lmk_ptr;
}

void Problem::addLandmarkList(LandmarkBaseList _lmk_list)
{
    getMapPtr()->addLandmarkList(_lmk_list);
}

StateBlockPtr Problem::addStateBlock(StateBlockPtr _state_ptr)
{
    //std::cout << "addStateBlockPtr" << std::endl;
    // add the state unit to the list
    state_block_list_.push_back(_state_ptr);
    // queue for solver manager
    state_block_notification_list_.push_back(StateBlockNotification({ADD,_state_ptr}));

    return _state_ptr;
}

void Problem::updateStateBlockPtr(StateBlockPtr _state_ptr)
{
    // queue for solver manager
    state_block_notification_list_.push_back(StateBlockNotification({UPDATE,_state_ptr}));
}

void Problem::removeStateBlockPtr(StateBlockPtr _state_ptr)
{
    // add the state unit to the list
    state_block_list_.remove(_state_ptr);

    // Check if the state addition is still as a notification
    auto state_found_it = state_block_notification_list_.end();
    for (auto state_notif_it = state_block_notification_list_.begin(); state_notif_it != state_block_notification_list_.end(); state_notif_it++)
    {
        if (state_notif_it->notification_ == ADD && state_notif_it->state_block_ptr_ == _state_ptr)
        {
            state_found_it = state_notif_it;
            break;
        }
    }
    // Remove addition notification
    if (state_found_it != state_block_notification_list_.end())
    	state_block_notification_list_.erase(state_found_it);
    // Add remove notification
    else
    	state_block_notification_list_.push_back(StateBlockNotification({REMOVE, nullptr, _state_ptr->getPtr()}));

}

ConstraintBasePtr Problem::addConstraintPtr(ConstraintBasePtr _constraint_ptr)
{
    //std::cout << "addConstraintPtr" << std::endl;
    // queue for solver manager
    constraint_notification_list_.push_back(ConstraintNotification({ADD, _constraint_ptr, _constraint_ptr->id()}));

    return _constraint_ptr;
}

void Problem::removeConstraintPtr(ConstraintBasePtr _constraint_ptr)
{
    // Check if the constraint addition is still as a notification
    auto ctr_found_it = constraint_notification_list_.end();
    for (auto ctr_notif_it = constraint_notification_list_.begin(); ctr_notif_it != constraint_notification_list_.end(); ctr_notif_it++)
    {
        if (ctr_notif_it->notification_ == ADD && ctr_notif_it->constraint_ptr_ == _constraint_ptr)
        {
            ctr_found_it = ctr_notif_it;
            break;
        }
    }
    // Remove addition notification
    if (ctr_found_it != constraint_notification_list_.end())
        constraint_notification_list_.erase(ctr_found_it);
    // Add remove notification
    else
        constraint_notification_list_.push_back(ConstraintNotification({REMOVE, nullptr, _constraint_ptr->id()}));
}

void Problem::clearCovariance()
{
    covariances_.clear();
}

void Problem::addCovarianceBlock(StateBlockPtr _state1, StateBlockPtr _state2, const Eigen::MatrixXs& _cov)
{
    assert(_state1->getSize() == (unsigned int ) _cov.rows() && "wrong covariance block size");
    assert(_state2->getSize() == (unsigned int ) _cov.cols() && "wrong covariance block size");

    covariances_[std::pair<StateBlockPtr, StateBlockPtr>(_state1, _state2)] = _cov;
}

bool Problem::getCovarianceBlock(StateBlockPtr _state1, StateBlockPtr _state2, Eigen::MatrixXs& _cov, const int _row,
                                 const int _col)
{
    //std::cout << "entire cov to be filled:" << std::endl << _cov << std::endl;
    //std::cout << "_row " << _row << std::endl;
    //std::cout << "_col " << _col << std::endl;
    //std::cout << "_state1 size: " << _state1->getSize() << std::endl;
    //std::cout << "_state2 size: " << _state2->getSize() << std::endl;
    //std::cout << "part of cov to be filled:" << std::endl <<  _cov.block(_row, _col, _state1->getSize(), _state2->getSize()) << std::endl;
    //if (covariances_.find(std::pair<StateBlockPtr, StateBlockPtr>(_state1, _state2)) != covariances_.end())
    //    std::cout << "stored cov" << std::endl << covariances_[std::pair<StateBlockPtr, StateBlockPtr>(_state1, _state2)] << std::endl;
    //else if (covariances_.find(std::pair<StateBlockPtr, StateBlockPtr>(_state2, _state1)) != covariances_.end())
    //    std::cout << "stored cov" << std::endl << covariances_[std::pair<StateBlockPtr, StateBlockPtr>(_state2, _state1)].transpose() << std::endl;

    assert(_row + _state1->getSize() <= _cov.rows() && _col + _state2->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");

    if (covariances_.find(std::pair<StateBlockPtr, StateBlockPtr>(_state1, _state2)) != covariances_.end())
        _cov.block(_row, _col, _state1->getSize(), _state2->getSize()) =
                covariances_[std::pair<StateBlockPtr, StateBlockPtr>(_state1, _state2)];
    else if (covariances_.find(std::pair<StateBlockPtr, StateBlockPtr>(_state2, _state1)) != covariances_.end())
       _cov.block(_row, _col, _state1->getSize(), _state2->getSize()) =
                covariances_[std::pair<StateBlockPtr, StateBlockPtr>(_state2, _state1)].transpose();
    else
        return false;

    return true;
}

bool Problem::getFrameCovariance(FrameBasePtr _frame_ptr, Eigen::MatrixXs& _covariance)
{
    return getCovarianceBlock(_frame_ptr->getPPtr(), _frame_ptr->getPPtr(), _covariance, 0, 0) &&
    getCovarianceBlock(_frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _covariance, 0,_frame_ptr->getPPtr()->getSize()) &&
    getCovarianceBlock(_frame_ptr->getOPtr(), _frame_ptr->getPPtr(), _covariance, _frame_ptr->getPPtr()->getSize(), 0) &&
    getCovarianceBlock(_frame_ptr->getOPtr(), _frame_ptr->getOPtr(), _covariance, _frame_ptr->getPPtr()->getSize() ,_frame_ptr->getPPtr()->getSize());
}

Eigen::MatrixXs Problem::getFrameCovariance(FrameBasePtr _frame_ptr)
{
    Eigen::MatrixXs covariance = Eigen::MatrixXs::Zero(_frame_ptr->getPPtr()->getSize()+_frame_ptr->getOPtr()->getSize(), _frame_ptr->getPPtr()->getSize()+_frame_ptr->getOPtr()->getSize());
    getFrameCovariance(_frame_ptr, covariance);
    return covariance;
}

bool Problem::getLandmarkCovariance(LandmarkBasePtr _landmark_ptr, Eigen::MatrixXs& _covariance)
{
    return getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getPPtr(), _covariance, 0, 0) &&
    getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getOPtr(), _covariance, 0,_landmark_ptr->getPPtr()->getSize()) &&
    getCovarianceBlock(_landmark_ptr->getOPtr(), _landmark_ptr->getPPtr(), _covariance, _landmark_ptr->getPPtr()->getSize(), 0) &&
    getCovarianceBlock(_landmark_ptr->getOPtr(), _landmark_ptr->getOPtr(), _covariance, _landmark_ptr->getPPtr()->getSize() ,_landmark_ptr->getPPtr()->getSize());
}

Eigen::MatrixXs Problem::getLandmarkCovariance(LandmarkBasePtr _landmark_ptr)
{
    Eigen::MatrixXs covariance = Eigen::MatrixXs::Zero(_landmark_ptr->getPPtr()->getSize()+_landmark_ptr->getOPtr()->getSize(), _landmark_ptr->getPPtr()->getSize()+_landmark_ptr->getOPtr()->getSize());
    getLandmarkCovariance(_landmark_ptr, covariance);
    return covariance;
}

MapBasePtr Problem::addMap(MapBasePtr _map_ptr)
{
    map_ptr_ = _map_ptr;
    map_ptr_->setProblem(shared_from_this());

    return map_ptr_;
}

TrajectoryBasePtr Problem::setTrajectory(TrajectoryBasePtr _trajectory_ptr)
{
    trajectory_ptr_ = _trajectory_ptr;
    trajectory_ptr_->setProblem(shared_from_this());

    return trajectory_ptr_;
}

MapBasePtr Problem::getMapPtr()
{
    return map_ptr_;
}

TrajectoryBasePtr Problem::getTrajectoryPtr()
{
    return trajectory_ptr_;
}

HardwareBasePtr Problem::getHardwarePtr()
{
    return hardware_ptr_;
}

FrameBasePtr Problem::getLastFramePtr()
{
    return trajectory_ptr_->getLastFramePtr();
}

FrameBasePtr Problem::getLastKeyFramePtr()
{
    return trajectory_ptr_->getLastKeyFramePtr();
}

StateBlockList& Problem::getStateBlockList()
{
    return state_block_list_;
}



void Problem::setOrigin(const Eigen::VectorXs& _origin_pose, const Eigen::MatrixXs& _origin_cov, const TimeStamp& _ts)
{
    if (!origin_is_set_)
    {
        // Create origin frame
        FrameBasePtr origin_frame_ptr = createFrame(KEY_FRAME, _origin_pose, _ts);
        // FIXME: create a fix sensor
        IntrinsicsBasePtr fix_instrinsics; // nullptr
        SensorBasePtr fix_sensor_ptr = installSensor("GPS", "initial pose", Eigen::VectorXs::Zero(3), fix_instrinsics );
        std::shared_ptr<CaptureFix> init_capture = std::make_shared<CaptureFix>(_ts, fix_sensor_ptr, _origin_pose, _origin_cov);
        origin_frame_ptr->addCapture(init_capture);
        init_capture->process();

        // notify processors about the new keyframe
        for (auto sensor_ptr : hardware_ptr_->getSensorList())
            for (auto processor_ptr : sensor_ptr->getProcessorList())
                if (processor_ptr->isMotion())
                    (std::static_pointer_cast<ProcessorMotion>(processor_ptr))->setOrigin(origin_frame_ptr);

        origin_is_set_ = true;
    }
    else
        throw std::runtime_error("Origin already setted!");
}

void Problem::loadMap(const std::string& _filename_dot_yaml)
{
    getMapPtr()->load(_filename_dot_yaml);
}

void Problem::saveMap(const std::string& _filename_dot_yaml, const std::string& _map_name)
{
    getMapPtr()->save(_filename_dot_yaml, _map_name);
}

void Problem::print(int depth, bool constr_by, bool metric, bool state_blocks)
{
    using std::cout;
    using std::endl;

    cout << endl;
    cout << "P: wolf tree status ---------------------" << endl;
    cout << "Hardware" << ((depth < 1) ? ("   -- " + std::to_string(getHardwarePtr()->getSensorList().size()) + "S") : "")  << endl;
    if (depth >= 1)
    {
        for (auto S : getHardwarePtr()->getSensorList())
        {
            cout << "  S" << S->id();
            if (depth < 2)
                cout << " -- " << S->getProcessorList().size() << "p";
            cout << endl;
            if (state_blocks)
            {
                cout << "    sb:";
                for (auto sb : S->getStateBlockVec())
                    if (sb != nullptr)
                        cout << " " << (sb->isFixed() ? "Fix" : "Est");
                cout << endl;
            }
            if (depth >= 2)
            {
                for (auto p : S->getProcessorList())
                {
                    if (p->isMotion())
                    {
                        std::cout << "    pm" << p->id() << std::endl;
                        ProcessorMotion::Ptr pm = std::static_pointer_cast<ProcessorMotion>(p);
                        if (pm->getOriginPtr())
                            cout << "      o: C" << pm->getOriginPtr()->id() << " - F"
                            << pm->getOriginPtr()->getFramePtr()->id() << endl;
                        if (pm->getLastPtr())
                            cout << "      l: C" << pm->getLastPtr()->id() << " - F"
                            << pm->getLastPtr()->getFramePtr()->id() << endl;
                        if (pm->getIncomingPtr())
                            cout << "      i: C" << pm->getIncomingPtr()->id() << endl;
                    }
                    else
                    {
                        try
                        {
                            cout << "    pt" << p->id() << endl;
                            ProcessorTracker::Ptr pt = std::static_pointer_cast<ProcessorTracker>(p);
                            if (pt->getOriginPtr())
                                cout << "      o: C" << pt->getOriginPtr()->id() << " - F"
                                << pt->getOriginPtr()->getFramePtr()->id() << endl;
                            if (pt->getLastPtr())
                                cout << "      l: C" << pt->getLastPtr()->id() << " - F"
                                << pt->getLastPtr()->getFramePtr()->id() << endl;
                            if (pt->getIncomingPtr())
                                cout << "      i: C" << pt->getIncomingPtr()->id() << endl;
                        }
                        catch (std::runtime_error& e)
                        {
                            cout << "Unknown processor type!" << endl;
                        }

                    }
                } // for p
            }
        } // for S
    }
    cout << "Trajectory" << ((depth < 1) ? (" -- " + std::to_string(getTrajectoryPtr()->getFrameList().size()) + "F") : "")  << endl;
    if (depth >= 1)
    {
        for (auto F : getTrajectoryPtr()->getFrameList())
        {
            cout << (F->isKey() ? "  KF" : "  F") << F->id() << ((depth < 2) ? " -- " + std::to_string(F->getCaptureList().size()) + "C  " : "");
            if (constr_by)
            {
                cout << "  <-- ";
                for (auto cby : F->getConstrainedByList())
                    cout << "c" << cby->id() << " \t";
            }
            cout << endl;
            if (metric)
            {
                cout << (F->isFixed() ? "    Fixed" : "    Estim") << ", ts=" << std::setprecision(5)
                        << F->getTimeStamp().get();
                cout << ",\t x = ( " << std::setprecision(2) << F->getState().transpose() << ")";
                cout << endl;
            }
            if (state_blocks)
            {
                cout << "    sb:";
                for (auto sb : F->getStateBlockVec())
                    if (sb != nullptr)
                        cout << " " << (sb->isFixed() ? "Fix" : "Est");
                cout << endl;
            }
            if (depth >= 2)
            {
                for (auto C : F->getCaptureList())
                {
                    cout << "    C" << C->id() << " -> S" << C->getSensorPtr()->id() << ((depth < 3) ? " -- " + std::to_string(C->getFeatureList().size()) + "f" : "") << endl;
                    if (depth >= 3)
                    {
                        for (auto f : C->getFeatureList())
                        {
                            cout << "      f" << f->id() << ((depth < 4) ? " -- " + std::to_string(f->getConstraintList().size()) + "c  " : "");
                            if (constr_by)
                            {
                                cout << "  <--\t";
                                for (auto cby : f->getConstrainedByList())
                                    cout << "c" << cby->id() << " \t";
                            }
                            cout << endl;
                            if (metric)
                                cout << "        m = ( " << std::setprecision(3) << f->getMeasurement().transpose()
                                        << ")" << endl;
                            if (depth >= 4)
                            {
                                for (auto c : f->getConstraintList())
                                {
                                    cout << "        c" << c->id() << " -->";
                                    if (c->getFrameOtherPtr() != nullptr && c->getFeatureOtherPtr() != nullptr && c->getLandmarkOtherPtr() != nullptr)
                                        cout << " A";
                                    if (c->getFrameOtherPtr() != nullptr)
                                        cout << " F" << c->getFrameOtherPtr()->id();
                                    if (c->getFeatureOtherPtr() != nullptr)
                                        cout << " f" << c->getFeatureOtherPtr()->id();
                                    if (c->getLandmarkOtherPtr() != nullptr)
                                        cout << " L" << c->getLandmarkOtherPtr()->id();
                                    cout << endl;
                                } // for c
                            }
                        } // for f
                    }
                } // for C
            }
        } // for F
    }
    cout << "Map" << ((depth < 1) ? ("        -- " + std::to_string(getMapPtr()->getLandmarkList().size()) + "L") : "") << endl;
    if (depth >= 1)
    {
        for (auto L : getMapPtr()->getLandmarkList())
        {
            cout << "  L" << L->id();
            if (constr_by)
            {
                cout << "\t<-- ";
                for (auto cby : L->getConstrainedByList())
                    cout << "c" << cby->id() << " \t";
            }
            cout << endl;
            if (state_blocks)
            {
                cout << "    sb:";
                for (auto sb : L->getStateBlockVec())
                    if (sb != nullptr)
                        cout << " " << (sb->isFixed() ? "Fix" : "Est");
                cout << endl;
            }
        } // for L
    }
    cout << "-----------------------------------------" << endl;
    cout << endl;
}

bool Problem::check(int verbose_level)
{
    bool is_consistent = true;
    std::cout << std::endl;
    std::cout << "Wolf tree integrity ---------------------" << std::endl;
    auto P_raw = this;
    auto H = hardware_ptr_;
    if (verbose_level > 0)
    {
        std::cout << "P @ " << P_raw << std::endl;
        std::cout << "H @ " << H.get() << std::endl;
    }
    is_consistent = is_consistent && (H->getProblem().get() == P_raw);
    for (auto S : H->getSensorList())
    {
        if (verbose_level > 0)
        {
            std::cout << "  S" << S->id() << " @ " << S.get() << std::endl;
            std::cout << "    -> P @ " << S->getProblem().get() << std::endl;
            std::cout << "    -> H @ " << S->getHardwarePtr().get() << std::endl;
        }
        is_consistent = is_consistent && (S->getProblem().get() == P_raw);
        is_consistent = is_consistent && (S->getHardwarePtr() == H);
        for (auto p : S->getProcessorList())
        {
            if (verbose_level > 0)
            {
                std::cout << "    p" << p->id() << " @ " << p.get() << " -> S" << p->getSensorPtr()->id() << std::endl;
                std::cout << "      -> P  @ " << p->getProblem().get() << std::endl;
                std::cout << "      -> S" << p->getSensorPtr()->id() << " @ " << p->getSensorPtr().get() << std::endl;
            }
            is_consistent = is_consistent && (p->getProblem().get() == P_raw);
            is_consistent = is_consistent && (p->getSensorPtr() == S);
        }
    }
    auto T = trajectory_ptr_;
    if (verbose_level > 0)
    {
        std::cout << "T @ " << T.get() << std::endl;
    }
    is_consistent = is_consistent && (T->getProblem().get() == P_raw);
    for (auto F : T->getFrameList())
    {
        if (verbose_level > 0)
        {
            std::cout << (F->isKey() ? "  KF" : "  F") << F->id() << " @ " << F.get() << std::endl;
            std::cout << "    -> P @ " << F->getProblem().get() << std::endl;
            std::cout << "    -> T @ " << F->getTrajectoryPtr().get() << std::endl;
            for (auto c : F->getConstrainedByList())
            {
                std::cout << "    <- c" << c->id() << " -> F" << c->getFrameOtherPtr()->id() << std::endl;
            }
        }
        is_consistent = is_consistent && (F->getProblem().get() == P_raw);
        is_consistent = is_consistent && (F->getTrajectoryPtr() == T);
        for (auto C : F->getCaptureList())
        {
            if (verbose_level > 0)
            {
                std::cout << "    C" << C->id() << " @" << C.get() << " -> S" << C->getSensorPtr()->id() << std::endl;
                std::cout << "      -> P  @ " << C->getProblem().get() << std::endl;
                std::cout << "      -> F" << C->getFramePtr()->id() << " @ " << C->getFramePtr().get() << std::endl;
            }
            is_consistent = is_consistent && (C->getProblem().get() == P_raw);
            is_consistent = is_consistent && (C->getFramePtr() == F);
            for (auto f : C->getFeatureList())
            {
                if (verbose_level > 0)
                {
                    std::cout << "      f" << f->id() << " @" << f.get() << std::endl;
                    std::cout << "        -> P  @ " << f->getProblem().get() << std::endl;
                    std::cout << "        -> C" << f->getCapturePtr()->id() << " @ " << f->getCapturePtr().get()
                            << std::endl;
                }
                is_consistent = is_consistent && (f->getProblem().get() == P_raw);
                is_consistent = is_consistent && (f->getCapturePtr() == C);

                if (verbose_level > 0)
                {
                    for (auto c : f->getConstrainedByList())
                    {
                        std::cout << "     <- c" << c->id() << " -> f" << c->getFeatureOtherPtr()->id() << std::endl;
                    }
                }
                for (auto c : f->getConstraintList())
                {
                    if (verbose_level > 0)
                        std::cout << "        c" << c->id() << " @" << C.get();

                    auto Fo = c->getFrameOtherPtr();
                    auto fo = c->getFeatureOtherPtr();
                    auto Lo = c->getLandmarkOtherPtr();

                    if ( !Fo && !fo && !Lo )    // case ABSOLUTE:
                    {
                        if (verbose_level > 0)
                            std::cout << " --> A" << std::endl;
                    }

                    if ( Fo )  // case FRAME:
                    {
                        if (verbose_level > 0)
                            std::cout << " --> F" << Fo->id() << " <- ";
                        bool found = false;
                        for (auto cby : Fo->getConstrainedByList())
                        {
                            if (verbose_level > 0)
                                std::cout << " c" << cby->id();
                            found = found || (c == cby);
                        }
                        if (verbose_level > 0)
                            std::cout << std::endl;
                        is_consistent = is_consistent && found;
                    }

                    if ( fo )   // case FEATURE:
                    {
                        if (verbose_level > 0)
                            std::cout << " --> f" << fo->id() << " <- ";
                        bool found = false;
                        for (auto cby : fo->getConstrainedByList())
                        {
                            if (verbose_level > 0)
                                std::cout << " c" << cby->id();
                            found = found || (c == cby);
                        }
                        if (verbose_level > 0)
                            std::cout << std::endl;
                        is_consistent = is_consistent && found;
                    }

                    if ( Lo )      // case LANDMARK:
                    {
                        if (verbose_level > 0)
                            std::cout << " --> L" << Lo->id() << " <- ";
                        bool found = false;
                        for (auto cby : Lo->getConstrainedByList())
                        {
                            if (verbose_level > 0)
                                std::cout << " c" << cby->id();
                            found = found || (c == cby);
                        }
                        if (verbose_level > 0)
                            std::cout << std::endl;
                        is_consistent = is_consistent && found;
                    }
                    if (verbose_level > 0)
                    {
                        std::cout << "          -> P  @ " << c->getProblem().get() << std::endl;
                        std::cout << "          -> f" << c->getFeaturePtr()->id() << " @ " << c->getFeaturePtr().get() << std::endl;
                    }
                    is_consistent = is_consistent && (c->getProblem().get() == P_raw);
                    is_consistent = is_consistent && (c->getFeaturePtr() == f);
                }
            }
        }
    }
    auto M = map_ptr_;
    if (verbose_level > 0)
        std::cout << "M @ " << M.get() << std::endl;
    is_consistent = is_consistent && (M->getProblem().get() == P_raw);
    for (auto L : M->getLandmarkList())
    {
        if (verbose_level > 0)
            std::cout << "  L" << L->id() << " @" << L.get() << std::endl;
        is_consistent = is_consistent && (L->getProblem().get() == P_raw);
        is_consistent = is_consistent && (L->getMapPtr() == M);
        for (auto cby : L->getConstrainedByList())
        {
            if (verbose_level > 0)
                std::cout << "      <- c" << cby->id() << " -> L" << cby->getLandmarkOtherPtr()->id() << std::endl;
            is_consistent = is_consistent && (cby->getLandmarkOtherPtr() && L->id() == cby->getLandmarkOtherPtr()->id());
        }
    }

    std::cout << "--------------------------- Wolf tree " << (is_consistent ? " OK" : "Not OK !!") << std::endl;
    std::cout << std::endl;

    return is_consistent;
}

} // namespace wolf
