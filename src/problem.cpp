#include "problem.h"
#include "constraint_base.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "hardware_base.h"
#include "trajectory_base.h"
#include "map_base.h"
#include "processor_motion.h"
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
        hardware_ptr_(),
        trajectory_ptr_(),
        map_ptr_(),
        processor_motion_ptr_(),
        origin_is_set_(false)
{
    hardware_ptr_ = std::make_shared<HardwareBase>();
    trajectory_ptr_ = std::make_shared<TrajectoryBase>(_frame_structure);
    map_ptr_ = std::make_shared<MapBase>();
}

void Problem::setup()
{
    hardware_ptr_->setProblem(shared_from_this());
    trajectory_ptr_->setProblem(shared_from_this());
    map_ptr_->setProblem(shared_from_this());
}

ProblemPtr Problem::create(FrameStructure _frame_structure)
{
    ProblemPtr p(std::make_shared<Problem>(_frame_structure));
    p->setup();
    return p;
}

Problem::~Problem()
{
    std::cout << "destructed -P" << std::endl;
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
    ProcessorBasePtr prc_ptr = ProcessorFactory::get().create(uppercase(_prc_type), _unique_processor_name, _prc_params);
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

FrameBasePtr Problem::createFrame(FrameKeyType _frame_type, const TimeStamp& _time_stamp)
{
    return createFrame(_frame_type, getStateAtTimeStamp(_time_stamp), _time_stamp);
}

FrameBasePtr Problem::createFrame(FrameKeyType _frame_key_type, const Eigen::VectorXs& _frame_state,
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
            std::cout << __FILE__ << ":" << __FUNCTION__ << "():" << __LINE__ << std::endl;
            return trajectory_ptr_->addFrame(std::make_shared<FrameBase>(_frame_key_type, _time_stamp, std::make_shared<StateBlock>(_frame_state.head(3)),
                                  std::make_shared<StateQuaternion>(_frame_state.segment<4>(3)),
                                  std::make_shared<StateBlock>(_frame_state.tail(3))));
        }
        case FRM_PVQBB_3D:
        {
            assert(_frame_state.size() == 16 && "Wrong state vector size");
            return trajectory_ptr_->addFrame(std::make_shared<FrameIMU>(_frame_key_type, _time_stamp, _frame_state));
        }
        default:
            throw std::runtime_error(
                    "Unknown frame structure. Add appropriate frame structure to the switch statement.");
    }
    return trajectory_ptr_->getLastFramePtr();
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
        case FRM_PVQBB_3D:
            return 16;
        default:
            throw std::runtime_error(
                    "Problem::getFrameStructureSize(): Unknown frame structure. Add appropriate frame structure to the switch statement.");
    }
}

Eigen::VectorXs Problem::zeroState()
{
    Eigen::VectorXs state = Eigen::VectorXs::Zero(getFrameStructureSize());
    if (trajectory_ptr_->getFrameStructure() == FRM_PO_3D || trajectory_ptr_->getFrameStructure() == FRM_POV_3D)
        state(6) = 1;
    if (trajectory_ptr_->getFrameStructure() == FRM_PVQBB_3D)
        state(9) = 1;
    return state;
}

bool Problem::permitKeyFrame(ProcessorBasePtr _processor_ptr)
{
    return true;
}

void Problem::keyFrameCallback(FrameBasePtr _keyframe_ptr, ProcessorBasePtr _processor_ptr, const Scalar& _time_tolerance)
{
    std::cout << __FILE__ << ":" << __FUNCTION__ << "():" << __LINE__ << std::endl;
    //std::cout << "Problem::keyFrameCallback: processor " << _processor_ptr->getName() << std::endl;
    for (auto sensor : hardware_ptr_->getSensorList())
    	for (auto processor : sensor->getProcessorList())
    		if (processor->id() != _processor_ptr->id())
                processor->keyFrameCallback(_keyframe_ptr, _time_tolerance);
    std::cout << __FILE__ << ":" << __FUNCTION__ << "():" << __LINE__ << std::endl;
}

LandmarkBasePtr Problem::addLandmark(LandmarkBasePtr _lmk_ptr)
{
    getMapPtr()->addLandmark(_lmk_ptr);
    return _lmk_ptr;
}

void Problem::addLandmarkList(LandmarkBaseList _lmk_list)
{
    //std::cout << "Problem::addLandmarkList" << std::endl;
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

void Problem::print()
{
    std::cout << std::endl;
    std::cout << "P: wolf tree status ---------------------" << std::endl;
    std::cout << "H" << std::endl;
    for (auto S : getHardwarePtr()->getSensorList() )
    {
        std::cout << "  S" << S->id() << std::endl;
        for (auto p : S->getProcessorList() )
        {
            std::cout << "    p" << p->id() << std::endl;
        }
    }
    std::cout << "T" << std::endl;
    for (auto F : getTrajectoryPtr()->getFrameList() )
    {
        std::cout << (F->isKey() ?  "  KF" : "  F") << F->id() << "  <--\t";
        for (auto cby : F->getConstrainedByList())
            std::cout << "c" << cby->id() << ",\t";
        std::cout << std::endl;
        std::cout << (F->isFixed() ?  "    Fixed" : "    Estim") << ", ts=" << std::setprecision(5) << F->getTimeStamp().get();
        std::cout << ",\t x = ( " << std::setprecision(2) << F->getState().transpose() << ")";
        //        std::cout << " T @ " << F->getTrajectoryPtr().get() << std::endl;
        std::cout << std::endl;
        for (auto C : F->getCaptureList() )
        {
            std::cout << "    C" << C->id() << " -> S" << C->getSensorPtr()->id() << std::endl;
            for (auto f : C->getFeatureList() )
            {
                std::cout << "      f" << f->id() << "  <--\t";
                for (auto cby : f->getConstrainedByList())
                    std::cout << "c" << cby->id() << ",\t";
                std::cout << std::endl;
                        std::cout << "        m = ( " << std::setprecision(3) << f->getMeasurement().transpose() << ")" << std::endl;
                for (auto c : f->getConstraintList() )
                {
                    std::cout << "        c" << c->id();
                    switch (c->getCategory())
                    {
                        case CTR_ABSOLUTE:
                            std::cout << " --> A" << std::endl;
                            break;
                        case CTR_FRAME:
                            std::cout << " --> F" << c->getFrameOtherPtr()->id() << std::endl;
                            break;
                        case CTR_FEATURE:
                            std::cout << " --> f" << c->getFeatureOtherPtr()->id() << std::endl;
                            break;
                        case CTR_LANDMARK:
                            std::cout << " --> L" << c->getLandmarkOtherPtr()->id() << std::endl;
                            break;
                    }
                }
            }
        }
    }
    std::cout << "M" << std::endl;
    for (auto L : getMapPtr()->getLandmarkList() )
    {
        std::cout << "  L" << L->id() << "\t<-- ";
        for (auto cby : L->getConstrainedByList())
            std::cout << "c" << cby->id() << ",\t";
        std::cout << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;
}

bool Problem::check()
{
    bool is_consistent = true;
    std::cout << std::endl;
    std::cout << "Wolf tree integrity ---------------------" << std::endl;
    auto P_raw = this;
    std::cout << "P @ " << P_raw << std::endl;
    auto H = hardware_ptr_;
    std::cout << "H @ " << H.get() << std::endl;
    is_consistent = is_consistent && (H->getProblem().get() == P_raw);
    for (auto S : H->getSensorList() )
    {
        std::cout << "  S" << S->id() << " @ " << S.get() << std::endl;
        std::cout << "    -> P @ " << S->getProblem().get() << std::endl;
        std::cout << "    -> H @ " << S->getHardwarePtr().get() << std::endl;
        is_consistent = is_consistent && (S->getProblem().get() == P_raw);
        is_consistent = is_consistent && (S->getHardwarePtr() == H);
        for (auto p : S->getProcessorList() )
        {
            std::cout << "    p" << p->id() << " @ " << p.get() << " -> S" << p->getSensorPtr()->id() << std::endl;
            std::cout << "      -> P  @ " << p->getProblem().get() << std::endl;
            std::cout << "      -> S" << p->getSensorPtr()->id() << " @ " << p->getSensorPtr().get() << std::endl;
            is_consistent = is_consistent && (p->getProblem().get() == P_raw);
            is_consistent = is_consistent && (p->getSensorPtr() == S);
        }
    }
    auto T = trajectory_ptr_;
    std::cout << "T @ " << T.get() << std::endl;
    is_consistent = is_consistent && (T->getProblem().get() == P_raw);
    for (auto F : T->getFrameList() )
    {
        std::cout << (F->isKey() ?  "  KF" : "  F") << F->id() << " @ " << F.get() << std::endl;
        std::cout << "    -> P @ " << F->getProblem().get() << std::endl;
        std::cout << "    -> T @ " << F->getTrajectoryPtr().get() << std::endl;
        is_consistent = is_consistent && (F->getProblem().get() == P_raw);
        is_consistent = is_consistent && (F->getTrajectoryPtr() == T);
        for (auto c : F->getConstrainedByList())
        {
            std::cout << "    <- c" << c->id() << " -> F" << c->getFrameOtherPtr()->id() << std::endl;
        }
        for (auto C : F->getCaptureList() )
        {
            std::cout << "    C" << C->id() << " @" << C.get() << " -> S" << C->getSensorPtr()->id() << std::endl;
            std::cout << "      -> P  @ " << C->getProblem().get() << std::endl;
            std::cout << "      -> F" << C->getFramePtr()->id() << " @ " << C->getFramePtr().get() << std::endl;
            is_consistent = is_consistent && (C->getProblem().get() == P_raw);
            is_consistent = is_consistent && (C->getFramePtr() == F);
            for (auto f : C->getFeatureList() )
            {
                std::cout << "      f" << f->id() << " @" << f.get() << std::endl;
                std::cout << "        -> P  @ " << f->getProblem().get() << std::endl;
                std::cout << "        -> C" << f->getCapturePtr()->id() << " @ " << f->getCapturePtr().get() << std::endl;
                is_consistent = is_consistent && (f->getProblem().get() == P_raw);
                is_consistent = is_consistent && (f->getCapturePtr() == C);

                for (auto c : f->getConstrainedByList())
                {
                    std::cout << "     <- c" << c->id() << " -> f" << c->getFeatureOtherPtr()->id() << std::endl;
                }
                for (auto c : f->getConstraintList() )
                {
                    std::cout << "        c" << c->id() << " @" << C.get();
                    switch (c->getCategory())
                    {
                        case CTR_ABSOLUTE:
                            std::cout << " --> A" << std::endl;
                            break;
                        case CTR_FRAME:
                            std::cout << " --> F" << c->getFrameOtherPtr()->id() << std::endl;
                            break;
                        case CTR_FEATURE:
                            std::cout << " --> f" << c->getFeatureOtherPtr()->id() << std::endl;
                            break;
                        case CTR_LANDMARK:
                            std::cout << " --> L" << c->getLandmarkOtherPtr()->id() << std::endl;
                            break;
                    }
                    std::cout << "          -> P  @ " << c->getProblem().get() << std::endl;
                    std::cout << "          -> f" << c->getFeaturePtr()->id() << " @ " << c->getFeaturePtr().get() << std::endl;
                    is_consistent = is_consistent && (c->getProblem().get() == P_raw);
                    is_consistent = is_consistent && (c->getFeaturePtr() == f);
                }
            }
        }
    }
    auto M = map_ptr_;
    std::cout << "M @ " << M.get() << std::endl;
    is_consistent = is_consistent && (M->getProblem().get() == P_raw);
    for (auto L : M->getLandmarkList() )
    {
        std::cout << "  L" << L->id() << " @" << L.get() << std::endl;
        is_consistent = is_consistent && (L->getProblem().get() == P_raw);
        is_consistent = is_consistent && (L->getMapPtr() == M);
        for (auto c : L->getConstrainedByList())
        {
            std::cout << "      <- c" << c->id() << " -> L" << c->getLandmarkOtherPtr()->id() << std::endl;
        }
    }

//    std::cout << std::endl;
    std::cout << "--------------------------- Wolf tree " << (is_consistent ? " OK" : "NOK") << std::endl;
    std::cout << std::endl;


    return is_consistent;
}

} // namespace wolf
