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

#include <algorithm>

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
        return installSensor(_sen_type, _unique_sensor_name, _extrinsics, IntrinsicsBasePtr());

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

ProcessorMotionPtr Problem::setProcessorMotion(const std::string& _processor_name)
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

void Problem::setProcessorMotion(ProcessorMotionPtr _processor_motion_ptr)
{
    processor_motion_ptr_ = _processor_motion_ptr;
}

void Problem::clearProcessorMotion()
{
    processor_motion_ptr_.reset();
}

FrameBasePtr Problem::emplaceFrame(FrameType _frame_type, const TimeStamp& _time_stamp)
{
    return emplaceFrame(_frame_type, getState(_time_stamp), _time_stamp);
}

FrameBasePtr Problem::emplaceFrame(FrameType _frame_key_type, const Eigen::VectorXs& _frame_state,
                                const TimeStamp& _time_stamp)
{
    assert(_frame_state.size() == getFrameStructureSize() && "Wrong state vector size");

    //std::cout << "Problem::createFrame" << std::endl;
    // ---------------------- CREATE NEW FRAME ---------------------
    // Create frame
    switch (trajectory_ptr_->getFrameStructure())
    {
        case FRM_PO_2D:
        {
            assert(_frame_state.size() == 3 && "Wrong state vector size. Use 3 for 2D pose.");
            return trajectory_ptr_->addFrame(FrameBase::create_PO_2D(_frame_key_type, _time_stamp, _frame_state));
//            return trajectory_ptr_->addFrame(std::make_shared<FrameBase>(_frame_key_type, _time_stamp, std::make_shared<StateBlock>(_frame_state.head(2)),
//                                  std::make_shared<StateBlock>(_frame_state.tail(1))));
        }
        case FRM_PO_3D:
        {
            assert(_frame_state.size() == 7 && "Wrong state vector size. Use 7 for 3D pose.");
            return trajectory_ptr_->addFrame(FrameBase::create_PO_3D(_frame_key_type, _time_stamp, _frame_state));
//           return trajectory_ptr_->addFrame(std::make_shared<FrameBase>(_frame_key_type, _time_stamp, std::make_shared<StateBlock>(_frame_state.head(3)),
//                                  std::make_shared<StateQuaternion>(_frame_state.tail(4))));
        }
        case FRM_POV_3D:
        {
            assert(_frame_state.size() == 10 && "Wrong state vector size. Use 10 for 3D pose and velocity.");
            return trajectory_ptr_->addFrame(FrameBase::create_POV_3D(_frame_key_type, _time_stamp, _frame_state));
//            return trajectory_ptr_->addFrame(std::make_shared<FrameBase>(_frame_key_type, _time_stamp, std::make_shared<StateBlock>(_frame_state.head(3)),
//                                  std::make_shared<StateQuaternion>(_frame_state.segment<4>(3)),
//                                  std::make_shared<StateBlock>(_frame_state.tail(3))));
        }
        case FRM_PQVBB_3D:
        {
            assert(_frame_state.size() == 16 && "Wrong state vector size. Use 16 for 3D pose, velocity, and IMU biases.");
            return trajectory_ptr_->addFrame(std::make_shared<FrameIMU>(_frame_key_type, _time_stamp, _frame_state));
        }
        default:
            throw std::runtime_error(
                    "Unknown frame structure. Add appropriate frame structure to the switch statement.");
    }
}

FrameBasePtr Problem::emplaceFrame(const std::string& _frame_structure, FrameType _frame_key_type,
                                   const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp)
{
    FrameBasePtr frm = FrameFactory::get().create(_frame_structure, _frame_key_type, _time_stamp, _frame_state);
    trajectory_ptr_->addFrame(frm);
    return frm;
}


Eigen::VectorXs Problem::getCurrentState()
{
    Eigen::VectorXs state(getFrameStructureSize());
    getCurrentState(state);
    return state;
}

Eigen::VectorXs Problem::getCurrentStateAndStamp(TimeStamp& ts)
{
    Eigen::VectorXs state(getFrameStructureSize());
    getCurrentStateAndStamp(state, ts);
    return state;
}

void Problem::getCurrentState(Eigen::VectorXs& state)
{
    assert(state.size() == getFrameStructureSize() && "Problem::getCurrentState: bad state size");

    if (processor_motion_ptr_ != nullptr)
        processor_motion_ptr_->getCurrentState(state);
    else if (trajectory_ptr_->getLastKeyFramePtr() != nullptr)
        trajectory_ptr_->getLastKeyFramePtr()->getState(state);
    else
        state = zeroState();
}


void Problem::getCurrentStateAndStamp(Eigen::VectorXs& state, TimeStamp& ts)
{
    assert(state.size() == getFrameStructureSize() && "Problem::getCurrentState: bad state size");

    if (processor_motion_ptr_ != nullptr)
        processor_motion_ptr_->getCurrentStateAndStamp(state, ts);
    else if (trajectory_ptr_->getLastKeyFramePtr() != nullptr)
    {
        trajectory_ptr_->getLastKeyFramePtr()->getTimeStamp(ts);
        trajectory_ptr_->getLastKeyFramePtr()->getState(state);
    }
    else
        state = zeroState();
}

void Problem::getState(const TimeStamp& _ts, Eigen::VectorXs& state)
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

Eigen::VectorXs Problem::getState(const TimeStamp& _ts)
{
    Eigen::VectorXs state(getFrameStructureSize());
    getState(_ts, state);
    return state;
}

Size Problem::getFrameStructureSize() const
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

void Problem::getFrameStructureSize(Size& _x_size, Size& _cov_size) const
{
    switch (trajectory_ptr_->getFrameStructure())
    {
        case FRM_PO_2D:
            _x_size = 3; _cov_size = 3;
            break;
        case FRM_PO_3D:
            _x_size = 7; _cov_size = 6;
            break;
        case FRM_POV_3D:
            _x_size = 10; _cov_size = 10;
            break;
        case FRM_PQVBB_3D:
            _x_size = 16; _cov_size = 15;
            break;
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
        case FRM_PO_2D:
            break;
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

void Problem::addLandmarkList(LandmarkBaseList& _lmk_list)
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
        constraint_notification_list_.erase(ctr_found_it); // FIXME see if the shared_ptr is still active and messing up
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

bool Problem::getCovarianceBlock(StateBlockList _st_list, Eigen::MatrixXs& _cov, const int _row, const int _col)
{

    std::map<StateBlockPtr, unsigned int> sb_2_idx;
    unsigned int next_idx = 0;
    for (auto st_it1 = _st_list.begin(); st_it1 != _st_list.end(); st_it1++)
        for (auto st_it2 = st_it1; st_it2 != _st_list.end(); st_it2++)
        {
            // add new key to map
            if (sb_2_idx.find(*st_it2) == sb_2_idx.end())
            {
                sb_2_idx[*st_it2] = next_idx;
                next_idx += (*st_it2)->getSize();
            }
            // search st1 & st2
            if (covariances_.find(std::pair<StateBlockPtr, StateBlockPtr>(*st_it1, *st_it2)) != covariances_.end())
            {
                assert(_row + sb_2_idx[*st_it1] + (*st_it1)->getSize() <= _cov.rows() &&
                       _col + sb_2_idx[*st_it2] + (*st_it2)->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");
                assert(_row + sb_2_idx[*st_it2] + (*st_it2)->getSize() <= _cov.rows() &&
                       _col + sb_2_idx[*st_it1] + (*st_it1)->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");

                _cov.block(_row + sb_2_idx[*st_it1], _col + sb_2_idx[*st_it2], (*st_it1)->getSize(), (*st_it2)->getSize()) = covariances_[std::pair<StateBlockPtr, StateBlockPtr>(*st_it1, *st_it2)];
                _cov.block(_row + sb_2_idx[*st_it2], _col + sb_2_idx[*st_it1], (*st_it2)->getSize(), (*st_it1)->getSize()) = covariances_[std::pair<StateBlockPtr, StateBlockPtr>(*st_it1, *st_it2)].transpose();
            }
            else if (covariances_.find(std::pair<StateBlockPtr, StateBlockPtr>(*st_it2, *st_it1)) != covariances_.end())
            {
                assert(_row + sb_2_idx[*st_it1] + (*st_it1)->getSize() <= _cov.rows() &&
                       _col + sb_2_idx[*st_it2] + (*st_it2)->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");
                assert(_row + sb_2_idx[*st_it2] + (*st_it2)->getSize() <= _cov.rows() &&
                       _col + sb_2_idx[*st_it1] + (*st_it1)->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");

                _cov.block(_row + sb_2_idx[*st_it1], _col + sb_2_idx[*st_it2], (*st_it1)->getSize(), (*st_it2)->getSize()) = covariances_[std::pair<StateBlockPtr, StateBlockPtr>(*st_it2, *st_it1)].transpose();
                _cov.block(_row + sb_2_idx[*st_it2], _col + sb_2_idx[*st_it1], (*st_it2)->getSize(), (*st_it1)->getSize()) = covariances_[std::pair<StateBlockPtr, StateBlockPtr>(*st_it2, *st_it1)];
            }
            else
                return false;
        }

    return true;
}

bool Problem::getFrameCovariance(FrameBasePtr _frame_ptr, Eigen::MatrixXs& _covariance)
{
//    return getCovarianceBlock(_frame_ptr->getPPtr(), _frame_ptr->getPPtr(), _covariance, 0,                                0                               ) &&
//           getCovarianceBlock(_frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _covariance, 0,                                _frame_ptr->getPPtr()->getSize()) &&
//           getCovarianceBlock(_frame_ptr->getOPtr(), _frame_ptr->getPPtr(), _covariance, _frame_ptr->getPPtr()->getSize(), 0                               ) &&
//           getCovarianceBlock(_frame_ptr->getOPtr(), _frame_ptr->getOPtr(), _covariance, _frame_ptr->getPPtr()->getSize(), _frame_ptr->getPPtr()->getSize());


    bool success(true);
    int i = 0, j = 0;

    for (auto sb_i : _frame_ptr->getStateBlockVec())
    {
        if (sb_i)
        {
            j = 0;
            for (auto sb_j : _frame_ptr->getStateBlockVec())
            {
                if (sb_j)
                {
                    success = success && getCovarianceBlock(sb_i, sb_j, _covariance, i, j);
                    j += sb_j->getSize();
                }
            }
            i += sb_i->getSize();
        }
    }
    return success;
}

Eigen::MatrixXs Problem::getFrameCovariance(FrameBasePtr _frame_ptr)
{
    Size sz = 0;
    for (auto sb : _frame_ptr->getStateBlockVec())
        if (sb)
            sz += sb->getSize();
    Eigen::MatrixXs covariance(sz, sz);

//    Eigen::MatrixXs covariance = Eigen::MatrixXs::Zero(_frame_ptr->getPPtr()->getSize()+_frame_ptr->getOPtr()->getSize(), _frame_ptr->getPPtr()->getSize()+_frame_ptr->getOPtr()->getSize());
    getFrameCovariance(_frame_ptr, covariance);
    return covariance;
}

Eigen::MatrixXs Problem::getLastKeyFrameCovariance()
{
    FrameBasePtr frm = getLastKeyFramePtr();
    return getFrameCovariance(frm);
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



FrameBasePtr Problem::setPrior(const Eigen::VectorXs& _prior_state, const Eigen::MatrixXs& _prior_cov, const TimeStamp& _ts)
{
    if (!origin_is_set_)
    {
        // Create origin frame
        FrameBasePtr origin_frame_ptr = emplaceFrame(KEY_FRAME, _prior_state, _ts);

        // create origin capture with the given state as data
        CaptureFixPtr init_capture = std::make_shared<CaptureFix>(_ts, nullptr, _prior_state, _prior_cov);
        origin_frame_ptr->addCapture(init_capture);

        // emplace feature and constraint
        init_capture->emplaceFeatureAndConstraint();

        // notify all motion processors about the origin keyframe
        for (auto sensor_ptr : hardware_ptr_->getSensorList())
            for (auto processor_ptr : sensor_ptr->getProcessorList())
                if (processor_ptr->isMotion())
                    (std::static_pointer_cast<ProcessorMotion>(processor_ptr))->setOrigin(origin_frame_ptr);

        origin_is_set_ = true;

        return origin_frame_ptr;
    }
    else
        throw std::runtime_error("Origin already set!");
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
        // Sensors
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
                // Processors
                for (auto p : S->getProcessorList())
                {
                    if (p->isMotion())
                    {
                        std::cout << "    pm" << p->id() << std::endl;
                        ProcessorMotionPtr pm = std::static_pointer_cast<ProcessorMotion>(p);
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
                            ProcessorTrackerPtr pt = std::static_pointer_cast<ProcessorTracker>(p);
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
        // Frames
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
                // Captures
                for (auto C : F->getCaptureList())
                {
                    cout << "    C" << C->id();
                    if (C->getSensorPtr()) cout << " -> S" << C->getSensorPtr()->id();
                    else cout << " -> S-";
                    cout << ((depth < 3) ? " -- " + std::to_string(C->getFeatureList().size()) + "f" : "") << endl;
                    if (depth >= 3)
                    {
                        // Features
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
                                // Constraints
                                for (auto c : f->getConstraintList())
                                {
                                    cout << "        c" << c->id() << " " << c->getType() << " -->";
                                    if (c->getFrameOtherPtr() == nullptr && c->getFeatureOtherPtr() == nullptr && c->getLandmarkOtherPtr() == nullptr)
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
        // Landmarks
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
    using std::cout;
    using std::endl;

    bool is_consistent = true; // true if all checks passed; false if any check fails.

    if (verbose_level) cout << endl;
    if (verbose_level) cout << "Wolf tree integrity ---------------------" << endl;
    auto P_raw = this;
    if (verbose_level > 0)
    {
        cout << "P @ " << P_raw << endl;
    }
    // ------------------------
    //     HARDWARE branch
    // ------------------------
    auto H = hardware_ptr_;
    if (verbose_level > 0)
    {
        cout << "H @ " << H.get() << endl;
    }
    // check pointer to Problem
    is_consistent = is_consistent && (H->getProblem().get() == P_raw);
    for (auto S : H->getSensorList())
    {
        if (verbose_level > 0)
        {
            cout << "  S" << S->id() << " @ " << S.get() << endl;
            cout << "    -> P @ " << S->getProblem().get() << endl;
            cout << "    -> H @ " << S->getHardwarePtr().get() << endl;
            for (auto sb : S->getStateBlockVec())
            {
                cout <<  "    sb @ " << sb.get();
                if (sb)
                {
                    auto lp = sb->getLocalParametrizationPtr();
                    if (lp)
                        cout <<  " (lp @ " << lp.get() << ")";
                }
                cout << endl;
            }
        }
        // check problem and hardware pointers
        is_consistent = is_consistent && (S->getProblem().get() == P_raw);
        is_consistent = is_consistent && (S->getHardwarePtr() == H);
        for (auto p : S->getProcessorList())
        {
            if (verbose_level > 0)
            {
                cout << "    p" << p->id() << " @ " << p.get() << " -> S" << p->getSensorPtr()->id() << endl;
                cout << "      -> P  @ " << p->getProblem().get() << endl;
                cout << "      -> S" << p->getSensorPtr()->id() << " @ " << p->getSensorPtr().get() << endl;
            }
            // check problem and sensor pointers
            is_consistent = is_consistent && (p->getProblem().get() == P_raw);
            is_consistent = is_consistent && (p->getSensorPtr() == S);
        }
    }
    // ------------------------
    //    TRAJECTORY branch
    // ------------------------
    auto T = trajectory_ptr_;
    if (verbose_level > 0)
    {
        cout << "T @ " << T.get() << endl;
    }
    // check pointer to Problem
    is_consistent = is_consistent && (T->getProblem().get() == P_raw);
    for (auto F : T->getFrameList())
    {
        if (verbose_level > 0)
        {
            cout << (F->isKey() ? "  KF" : "  F") << F->id() << " @ " << F.get() << endl;
            cout << "    -> P @ " << F->getProblem().get() << endl;
            cout << "    -> T @ " << F->getTrajectoryPtr().get() << endl;
            for (auto sb : F->getStateBlockVec())
            {
                cout <<  "    sb @ " << sb.get();
                if (sb)
                {
                    auto lp = sb->getLocalParametrizationPtr();
                    if (lp)
                        cout <<  " (lp @ " << lp.get() << ")";
                }
                cout << endl;
            }
        }
        // check problem and trajectory pointers
        is_consistent = is_consistent && (F->getProblem().get() == P_raw);
        is_consistent = is_consistent && (F->getTrajectoryPtr() == T);
        for (auto cby : F->getConstrainedByList())
        {
            if (verbose_level > 0)
            {
                cout << "    <- c" << cby->id() << " -> F" << cby->getFrameOtherPtr()->id() << endl;
            }
            // check constrained_by pointer to this frame
            is_consistent = is_consistent && (cby->getFrameOtherPtr() == F);
            for (auto sb : cby->getStateBlockPtrVector())
            {
                if (verbose_level > 0)
                {
                    cout << "      sb @ " << sb.get();
                    if (sb)
                    {
                        auto lp = sb->getLocalParametrizationPtr();
                        if (lp)
                            cout <<  " (lp @ " << lp.get() << ")";
                    }
                    cout << endl;
                }
            }
        }
        for (auto C : F->getCaptureList())
        {
            if (verbose_level > 0)
            {
                cout << "    C" << C->id() << " @ " << C.get() << " -> S";
                if (C->getSensorPtr()) cout << C->getSensorPtr()->id();
                else cout << "-";
                cout << endl;
                cout << "      -> P  @ " << C->getProblem().get() << endl;
                cout << "      -> F" << C->getFramePtr()->id() << " @ " << C->getFramePtr().get() << endl;
            }
            // check problem and frame pointers
            is_consistent = is_consistent && (C->getProblem().get() == P_raw);
            is_consistent = is_consistent && (C->getFramePtr() == F);
            for (auto f : C->getFeatureList())
            {
                if (verbose_level > 0)
                {
                    cout << "      f" << f->id() << " @ " << f.get() << endl;
                    cout << "        -> P  @ " << f->getProblem().get() << endl;
                    cout << "        -> C" << f->getCapturePtr()->id() << " @ " << f->getCapturePtr().get()
                            << endl;
                }
                // check problem and capture pointers
                is_consistent = is_consistent && (f->getProblem().get() == P_raw);
                is_consistent = is_consistent && (f->getCapturePtr() == C);

                for (auto cby : f->getConstrainedByList())
                {
                    if (verbose_level > 0)
                    {
                        cout << "     <- c" << cby->id() << " -> f" << cby->getFeatureOtherPtr()->id() << endl;
                    }
                    // check constrained_by pointer to this feature
                    is_consistent = is_consistent && (cby->getFeatureOtherPtr() == f);
                }
                for (auto c : f->getConstraintList())
                {
                    if (verbose_level > 0)
                        cout << "        c" << c->id() << " @ " << c.get();

                    auto Fo = c->getFrameOtherPtr();
                    auto fo = c->getFeatureOtherPtr();
                    auto Lo = c->getLandmarkOtherPtr();

                    if ( !Fo && !fo && !Lo )    // case ABSOLUTE:
                    {
                        if (verbose_level > 0)
                            cout << " --> Abs." << endl;
                    }

                    // find constrained_by pointer in constrained frame
                    if ( Fo )  // case FRAME:
                    {
                        if (verbose_level > 0)
                            cout << " --> F" << Fo->id() << " <- ";
                        bool found = false;
                        for (auto cby : Fo->getConstrainedByList())
                        {
                            if (verbose_level > 0)
                                cout << " c" << cby->id();
                            found = found || (c == cby);
                        }
                        if (verbose_level > 0)
                            cout << endl;
                        // check constrained_by pointer in constrained frame
                        is_consistent = is_consistent && found;
                    }

                    // find constrained_by pointer in constrained feature
                    if ( fo )   // case FEATURE:
                    {
                        if (verbose_level > 0)
                            cout << " --> f" << fo->id() << " <- ";
                        bool found = false;
                        for (auto cby : fo->getConstrainedByList())
                        {
                            if (verbose_level > 0)
                                cout << " c" << cby->id();
                            found = found || (c == cby);
                        }
                        if (verbose_level > 0)
                            cout << endl;
                        // check constrained_by pointer in constrained feature
                        is_consistent = is_consistent && found;
                    }

                    // find constrained_by pointer in constrained landmark
                    if ( Lo )      // case LANDMARK:
                    {
                        if (verbose_level > 0)
                            cout << " --> L" << Lo->id() << " <- ";
                        bool found = false;
                        for (auto cby : Lo->getConstrainedByList())
                        {
                            if (verbose_level > 0)
                                cout << " c" << cby->id();
                            found = found || (c == cby);
                        }
                        if (verbose_level > 0)
                            cout << endl;
                        // check constrained_by pointer in constrained landmark
                        is_consistent = is_consistent && found;
                    }
                    if (verbose_level > 0)
                    {
                        cout << "          -> P  @ " << c->getProblem().get() << endl;
                        cout << "          -> f" << c->getFeaturePtr()->id() << " @ " << c->getFeaturePtr().get() << endl;
                    }
                    // check problem and feature pointers
                    is_consistent = is_consistent && (c->getProblem().get() == P_raw);
                    is_consistent = is_consistent && (c->getFeaturePtr() == f);

                    // find state block pointers in all constrained nodes
                    SensorBasePtr S = c->getFeaturePtr()->getCapturePtr()->getSensorPtr(); // get own sensor to check sb
                    for (auto sb : c->getStateBlockPtrVector())
                    {
                        bool found = false;
                        if (verbose_level > 0)
                        {
                            cout <<  "          sb @ " << sb.get();
                            if (sb)
                            {
                                auto lp = sb->getLocalParametrizationPtr();
                                if (lp)
                                    cout <<  " (lp @ " << lp.get() << ")";
                            }
                        }
                        // find in own Frame
                        found = found || (std::find(F->getStateBlockVec().begin(), F->getStateBlockVec().end(), sb) != F->getStateBlockVec().end());
                        // find in own Sensor
                        if (S)
                            found = found || (std::find(S->getStateBlockVec().begin(), S->getStateBlockVec().end(), sb) != S->getStateBlockVec().end());
                        // find in constrained Frame
                        if (Fo)
                            found = found || (std::find(Fo->getStateBlockVec().begin(), Fo->getStateBlockVec().end(), sb) != Fo->getStateBlockVec().end());
                        if (fo)
                        {
                            // find in constrained feature's Frame
                            FrameBasePtr foF = fo->getFramePtr();
                            found = found || (std::find(foF->getStateBlockVec().begin(), foF->getStateBlockVec().end(), sb) != foF->getStateBlockVec().end());
                            // find in constrained feature's Sensor
                            SensorBasePtr foS = fo->getCapturePtr()->getSensorPtr();
                            found = found || (std::find(foS->getStateBlockVec().begin(), foS->getStateBlockVec().end(), sb) != foS->getStateBlockVec().end());
                        }
                        if (Lo)
                            // find in constrained landmark
                            found = found || (std::find(Lo->getStateBlockVec().begin(), Lo->getStateBlockVec().end(), sb) != Lo->getStateBlockVec().end());
                        if (verbose_level > 0)
                        {
                            if (found)
                                cout << " found";
                            else
                                cout << " NOT FOUND !";
                            cout << endl;
                        }
                        // check that all state block pointers were found
                        is_consistent = is_consistent && found;
                    }
                }
            }
        }
    }
    // ------------------------
    //       MAP branch
    // ------------------------
    auto M = map_ptr_;
    if (verbose_level > 0)
        cout << "M @ " << M.get() << endl;
    // check pointer to Problem
    is_consistent = is_consistent && (M->getProblem().get() == P_raw);
    for (auto L : M->getLandmarkList())
    {
        if (verbose_level > 0)
        {
            cout << "  L" << L->id() << " @ " << L.get() << endl;
            cout << "  -> P @ " << L->getProblem().get() << endl;
            cout << "  -> M @ " << L->getMapPtr().get() << endl;
            for (auto sb : L->getStateBlockVec())
            {
                cout <<  "  sb @ " << sb.get();
                if (sb)
                {
                    auto lp = sb->getLocalParametrizationPtr();
                    if (lp)
                        cout <<  " (lp @ " << lp.get() << ")";
                }
                cout << endl;
            }
        }
        // check problem and map pointers
        is_consistent = is_consistent && (L->getProblem().get() == P_raw);
        is_consistent = is_consistent && (L->getMapPtr() == M);
        for (auto cby : L->getConstrainedByList())
        {
            if (verbose_level > 0)
                cout << "      <- c" << cby->id() << " -> L" << cby->getLandmarkOtherPtr()->id() << endl;
            // check constrained_by pointer to this landmark
            is_consistent = is_consistent && (cby->getLandmarkOtherPtr() && L->id() == cby->getLandmarkOtherPtr()->id());
            for (auto sb : cby->getStateBlockPtrVector())
            {
                if (verbose_level > 0)
                {
                    cout << "      sb @ " << sb.get();
                    if (sb)
                    {
                        auto lp = sb->getLocalParametrizationPtr();
                        if (lp)
                            cout <<  " (lp @ " << lp.get() << ")";
                    }
                    cout << endl;
                }
            }
        }
    }

    if (verbose_level) cout << "--------------------------- Wolf tree " << (is_consistent ? " OK" : "Not OK !!") << endl;
    if (verbose_level) cout << endl;

    return is_consistent;
}

} // namespace wolf
