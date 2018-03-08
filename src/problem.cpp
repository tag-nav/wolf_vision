#include <sensor_GPS.h>
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
#include "factory.h"
#include "sensor_factory.h"
#include "processor_factory.h"

#include <algorithm>
#include "capture_pose.h"

namespace wolf
{

// unnamed namespace used for helper functions local to this file.
namespace
{
std::string uppercase(std::string s) {for (auto & c: s) c = std::toupper(c); return s;}
}


Problem::Problem(const std::string& _frame_structure) :
        hardware_ptr_(std::make_shared<HardwareBase>()),
        trajectory_ptr_(std::make_shared<TrajectoryBase>(_frame_structure)),
        map_ptr_(std::make_shared<MapBase>()),
        processor_motion_ptr_(),
        prior_is_set_(false),
        state_size_(0),
        state_cov_size_(0)
{
    if (_frame_structure == "PO 2D")
    {
        state_size_ = 3;
        state_cov_size_ = 3;
    }

    else if (_frame_structure == "PO 3D")
    {
        state_size_ = 7;
        state_cov_size_ = 6;
    }
    else if (_frame_structure == "POV 3D")
    {
        state_size_ = 10;
        state_cov_size_ = 9;
    }
    else std::runtime_error(
            "Problem::Problem(): Unknown frame structure. Add appropriate frame structure to the switch statement.");

}

void Problem::setup()
{
    hardware_ptr_->setProblem(shared_from_this());
    trajectory_ptr_->setProblem(shared_from_this());
    map_ptr_->setProblem(shared_from_this());
}

ProblemPtr Problem::create(const std::string& _frame_structure)
{
    ProblemPtr p(new Problem(_frame_structure)); // We use `new` and not `make_shared` since the Problem constructor is private and cannot be passed to `make_shared`.
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
    if (_corresponding_sensor_ptr == nullptr)
    {
      WOLF_ERROR("Cannot install processor '", _unique_processor_name,
                 "' since the associated sensor does not exist !");
      return ProcessorBasePtr();
    }

    ProcessorBasePtr prc_ptr = ProcessorFactory::get().create(uppercase(_prc_type), _unique_processor_name, _prc_params, _corresponding_sensor_ptr);
    _corresponding_sensor_ptr->addProcessor(prc_ptr);

    // setting the origin in all processor motion if origin already setted
    if (prc_ptr->isMotion() && prior_is_set_)
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


FrameBasePtr Problem::emplaceFrame(const std::string& _frame_structure, //
                                   FrameType _frame_key_type, //
                                   const Eigen::VectorXs& _frame_state, //
                                   const TimeStamp& _time_stamp)
{
    FrameBasePtr frm = FrameFactory::get().create(_frame_structure, _frame_key_type, _time_stamp, _frame_state);
    trajectory_ptr_->addFrame(frm);
    return frm;
}

FrameBasePtr Problem::emplaceFrame(const std::string& _frame_structure, //
                                   FrameType _frame_key_type, //
                                   const TimeStamp& _time_stamp)
{
    return emplaceFrame(_frame_structure, _frame_key_type, getState(_time_stamp), _time_stamp);
}

FrameBasePtr Problem::emplaceFrame(FrameType _frame_key_type, //
                                   const Eigen::VectorXs& _frame_state, //
                                   const TimeStamp& _time_stamp)
{
    return emplaceFrame(trajectory_ptr_->getFrameStructure(), _frame_key_type, _frame_state, _time_stamp);
}

FrameBasePtr Problem::emplaceFrame(FrameType _frame_key_type, //
                                   const TimeStamp& _time_stamp)
{
    return emplaceFrame(trajectory_ptr_->getFrameStructure(), _frame_key_type, _time_stamp);
}

Eigen::VectorXs Problem::getCurrentState()
{
    Eigen::VectorXs state(getFrameStructureSize());
    getCurrentState(state);
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
    {
        processor_motion_ptr_->getCurrentState(state);
        processor_motion_ptr_->getCurrentTimeStamp(ts);
    }
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
    return state_size_;
}

void Problem::getFrameStructureSize(Size& _x_size, Size& _cov_size) const
{
    _x_size = state_size_;
    _cov_size = state_cov_size_;
}

Eigen::VectorXs Problem::zeroState()
{
    Eigen::VectorXs state = Eigen::VectorXs::Zero(getFrameStructureSize());

    // Set the quaternion identity for 3D states. Set only the real part to 1:
    if (trajectory_ptr_->getFrameStructure() == "PO 3D" ||
        trajectory_ptr_->getFrameStructure() == "POV 3D")
        state(6) = 1.0;

    return state;
}

bool Problem::permitKeyFrame(ProcessorBasePtr _processor_ptr)
{
    return true;
}

void Problem::keyFrameCallback(FrameBasePtr _keyframe_ptr, ProcessorBasePtr _processor_ptr, const Scalar& _time_tolerance)
{
    if (_processor_ptr)
    {
        WOLF_DEBUG((_processor_ptr->isMotion() ? "PM " : "PT "), _processor_ptr->getName(), ": KF", _keyframe_ptr->id(), " Callback emitted with ts = ", _keyframe_ptr->getTimeStamp());
    }
    else
    {
        WOLF_DEBUG("External callback: KF", _keyframe_ptr->id(), " Callback emitted with ts = ", _keyframe_ptr->getTimeStamp());
    }

    for (auto sensor : hardware_ptr_->getSensorList())
        for (auto processor : sensor->getProcessorList())
            if (processor && (processor != _processor_ptr) )
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
    //std::cout << "Problem::addStateBlockPtr " << _state_ptr.get() << std::endl;

    // add the state unit to the list
    state_block_list_.push_back(_state_ptr);
    // queue for solver manager
    state_block_notification_list_.push_back(StateBlockNotification({ADD,_state_ptr}));
    return _state_ptr;
}

void Problem::updateStateBlockPtr(StateBlockPtr _state_ptr)
{
    //std::cout << "Problem::updateStateBlockPtr " << _state_ptr.get() << std::endl;

    // queue for solver manager
    state_block_notification_list_.push_back(StateBlockNotification({UPDATE,_state_ptr}));
}

void Problem::removeStateBlockPtr(StateBlockPtr _state_ptr)
{
    //std::cout << "Problem::removeStateBlockPtr " << _state_ptr.get() << std::endl;

    // add the state unit to the list
    state_block_list_.remove(_state_ptr);

    // Check if the state addition is still as a notification
    auto state_notif_it = state_block_notification_list_.begin();
    for (; state_notif_it != state_block_notification_list_.end(); state_notif_it++)
        if (state_notif_it->notification_ == ADD && state_notif_it->state_block_ptr_ == _state_ptr)
            break;

    // Remove addition notification
    if (state_notif_it != state_block_notification_list_.end())
    	state_block_notification_list_.erase(state_notif_it);
    // Add remove notification
    else
    	state_block_notification_list_.push_back(StateBlockNotification({REMOVE, _state_ptr}));

}

ConstraintBasePtr Problem::addConstraintPtr(ConstraintBasePtr _constraint_ptr)
{
    //std::cout << "Problem::addConstraintPtr " << _constraint_ptr->id() << std::endl;
    // queue for solver manager
    constraint_notification_list_.push_back(ConstraintNotification({ADD, _constraint_ptr}));

    return _constraint_ptr;
}

void Problem::removeConstraintPtr(ConstraintBasePtr _constraint_ptr)
{
    //std::cout << "Problem::removeConstraintPtr " << _constraint_ptr->id() << std::endl;

    // Check if the constraint addition is still as a notification
    auto ctr_notif_it = constraint_notification_list_.begin();
    for (; ctr_notif_it != constraint_notification_list_.end(); ctr_notif_it++)
        if (ctr_notif_it->notification_ == ADD && ctr_notif_it->constraint_ptr_ == _constraint_ptr)
            break;

    // Remove addition notification
    if (ctr_notif_it != constraint_notification_list_.end())
        constraint_notification_list_.erase(ctr_notif_it); // CHECKED shared_ptr is not active after erase
    // Add remove notification
    else
        constraint_notification_list_.push_back(ConstraintNotification({REMOVE, _constraint_ptr}));
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

void Problem::addCovarianceBlock(StateBlockPtr _state1, const Eigen::MatrixXs& _cov)
{
    assert(_state1->getSize() == (unsigned int ) _cov.rows() && "wrong covariance block size");
    assert(_state1->getSize() == (unsigned int ) _cov.cols() && "wrong covariance block size");

    covariances_[std::make_pair(_state1, _state1)] = _cov;
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
    {
      WOLF_DEBUG("Could not find the requested covariance block.");
      return false;
    }

    return true;
}

bool Problem::getCovarianceBlock(std::map<StateBlockPtr, unsigned int> _sb_2_idx, Eigen::MatrixXs& _cov)
{
    // fill covariance
    for (auto it1 = _sb_2_idx.begin(); it1 != _sb_2_idx.end(); it1++)
        for (auto it2 = it1; it2 != _sb_2_idx.end(); it2++)
        {
            StateBlockPtr sb1 = it1->first;
            StateBlockPtr sb2 = it2->first;
            std::pair<StateBlockPtr, StateBlockPtr> pair_12(sb1, sb2);
            std::pair<StateBlockPtr, StateBlockPtr> pair_21(sb2, sb1);

            // search st1 & st2
            if (covariances_.find(pair_12) != covariances_.end())
            {
                assert(_sb_2_idx[sb1] + sb1->getSize() <= _cov.rows() &&
                       _sb_2_idx[sb2] + sb2->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");
                assert(_sb_2_idx[sb2] + sb2->getSize() <= _cov.rows() &&
                       _sb_2_idx[sb1] + sb1->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");

                _cov.block(_sb_2_idx[sb1], _sb_2_idx[sb2], sb1->getSize(), sb2->getSize()) = covariances_[pair_12];
                _cov.block(_sb_2_idx[sb2], _sb_2_idx[sb1], sb2->getSize(), sb1->getSize()) = covariances_[pair_12].transpose();
            }
            else if (covariances_.find(pair_21) != covariances_.end())
            {
                assert(_sb_2_idx[sb1] + sb1->getSize() <= _cov.rows() &&
                       _sb_2_idx[sb2] + sb2->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");
                assert(_sb_2_idx[sb2] + sb2->getSize() <= _cov.rows() &&
                       _sb_2_idx[sb1] + sb1->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");

                _cov.block(_sb_2_idx[sb1], _sb_2_idx[sb2], sb1->getSize(), sb2->getSize()) = covariances_[pair_21].transpose();
                _cov.block(_sb_2_idx[sb2], _sb_2_idx[sb1], sb2->getSize(), sb1->getSize()) = covariances_[pair_21];
            }
            else
                return false;
        }

    return true;
}

bool Problem::getCovarianceBlock(StateBlockPtr _state, Eigen::MatrixXs& _cov, const int _row_and_col)
{
    return getCovarianceBlock(_state, _state, _cov, _row_and_col, _row_and_col);
}

bool Problem::getFrameCovariance(FrameBaseConstPtr _frame_ptr, Eigen::MatrixXs& _covariance)
{
    bool success(true);
    int i = 0, j = 0;

    const auto& state_bloc_vec = _frame_ptr->getStateBlockVec();

    for (const auto& sb_i : state_bloc_vec)
    {
        if (sb_i)
        {
            j = 0;
            for (const auto& sb_j : state_bloc_vec)
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

Eigen::MatrixXs Problem::getFrameCovariance(FrameBaseConstPtr _frame_ptr)
{
    Size sz = 0;
    for (const auto& sb : _frame_ptr->getStateBlockVec())
        if (sb)
            sz += sb->getSize();

    Eigen::MatrixXs covariance(sz, sz);

    getFrameCovariance(_frame_ptr, covariance);
    return covariance;
}

Eigen::MatrixXs Problem::getLastKeyFrameCovariance()
{
    FrameBasePtr frm = getLastKeyFramePtr();
    return getFrameCovariance(frm);
}

bool Problem::getLandmarkCovariance(LandmarkBaseConstPtr _landmark_ptr, Eigen::MatrixXs& _covariance)
{
    bool success(true);
    int i = 0, j = 0;

    const auto& state_bloc_vec = _landmark_ptr->getStateBlockVec();

    for (const auto& sb_i : state_bloc_vec)
    {
        if (sb_i)
        {
            j = 0;
            for (const auto& sb_j : state_bloc_vec)
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

Eigen::MatrixXs Problem::getLandmarkCovariance(LandmarkBaseConstPtr _landmark_ptr)
{
    Size sz = 0;
    for (const auto& sb : _landmark_ptr->getStateBlockVec())
        if (sb)
            sz += sb->getSize();

    Eigen::MatrixXs covariance(sz, sz);

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



FrameBasePtr Problem::setPrior(const Eigen::VectorXs& _prior_state, const Eigen::MatrixXs& _prior_cov, const TimeStamp& _ts, const Scalar _time_tolerance)
{
    if ( ! prior_is_set_ )
    {
        // Create origin frame
        FrameBasePtr origin_keyframe = emplaceFrame(KEY_FRAME, _prior_state, _ts);

        // create origin capture with the given state as data
        // Capture fix only takes 3D position and Quaternion orientation
        CapturePosePtr init_capture;
        if (trajectory_ptr_->getFrameStructure() == "POV 3D")
            init_capture = std::make_shared<CapturePose>(_ts, nullptr, _prior_state.head(7), _prior_cov.topLeftCorner(6,6));
        else
            init_capture = std::make_shared<CapturePose>(_ts, nullptr, _prior_state, _prior_cov);

        origin_keyframe->addCapture(init_capture);

        // emplace feature and constraint
        init_capture->emplaceFeatureAndConstraint();

        // Notify all processors about the prior KF
        for (auto sensor : hardware_ptr_->getSensorList())
            for (auto processor : sensor->getProcessorList())
                if (processor->isMotion())
                    // Motion processors will set its origin at the KF
                    (std::static_pointer_cast<ProcessorMotion>(processor))->setOrigin(origin_keyframe);

        prior_is_set_ = true;

        // Notify all other processors about the origin KF --> they will join it or not depending on their received data
        for (auto sensor : hardware_ptr_->getSensorList())
            for (auto processor : sensor->getProcessorList())
                if ( !processor->isMotion() )
                    processor->keyFrameCallback(origin_keyframe, _time_tolerance);

        return origin_keyframe;
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
            cout << "  S" << S->id() << " " << S->getType();
            if (!metric && !state_blocks) cout << (S->isExtrinsicDynamic() ? " [Dyn," : " [Sta,") << (S->isIntrinsicDynamic() ? "Dyn]" : "Sta]");
            if (depth < 2)
                cout << " -- " << S->getProcessorList().size() << "p";
            cout << endl;
            if (metric && state_blocks)
            {
                for (auto i = 0; i < S->getStateBlockVec().size(); i++)
                {
                    if (i==0) cout << "    Extr " << (S->isExtrinsicDynamic() ? "[Dyn]" : "[Sta]") << " = [";
                    if (i==2) cout << "    Intr " << (S->isIntrinsicDynamic() ? "[Dyn]" : "[Sta]") << " = [";
                    auto sb = S->getStateBlockPtrDynamic(i);
                    if (sb)
                    {
                        cout << (sb->isFixed() ? " Fix( " : " Est( ") << sb->getState().transpose() << " )";
                    }
                    if (i==1) cout << " ]" << endl;
                }
                if (S->getStateBlockVec().size() > 2) cout << " ]" << endl;
            }
            else if (metric)
            {
                cout << "    Extr " << (S->isExtrinsicDynamic() ? "[Dyn]" : "[Sta]") << "= ( ";
                if (S->getPPtr())
                    cout << S->getPPtr()->getState().transpose();
                if (S->getOPtr())
                    cout << " " << S->getOPtr()->getState().transpose();
                cout  << " )";
                if (S->getIntrinsicPtr())
                    cout << "    Intr " << (S->isIntrinsicDynamic() ? "[Dyn]" : "[Sta]") << "= ( " << S->getIntrinsicPtr()->getState().transpose() << " )";
                cout << endl;
            }
            else if (state_blocks)
            {
                cout << "    sb:" << (S->isExtrinsicDynamic() ? "[Dyn," : "[Sta,") << (S->isIntrinsicDynamic() ? "Dyn]" : "Sta]");
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
                        std::cout << "    pm" << p->id() << " " << p->getType() << endl;
                        ProcessorMotionPtr pm = std::static_pointer_cast<ProcessorMotion>(p);
                        if (pm->getOriginPtr())
                            cout << "      o: C" << pm->getOriginPtr()->id() << " - " << (pm->getOriginPtr()->getFramePtr()->isKey() ? "  KF" : "  F")
                            << pm->getOriginPtr()->getFramePtr()->id() << endl;
                        if (pm->getLastPtr())
                            cout << "      l: C" << pm->getLastPtr()->id() << " - " << (pm->getLastPtr()->getFramePtr()->isKey() ? "  KF" : "  F")
                            << pm->getLastPtr()->getFramePtr()->id() << endl;
                        if (pm->getIncomingPtr())
                            cout << "      i: C" << pm->getIncomingPtr()->id() << endl;
                    }
                    else
                    {
                        cout << "    pt" << p->id() << " " << p->getType() << endl;
                        ProcessorTrackerPtr pt = std::dynamic_pointer_cast<ProcessorTracker>(p);
                        if (pt)
                        {
                            if (pt->getOriginPtr())
                                cout << "      o: C" << pt->getOriginPtr()->id() << " - " << (pt->getOriginPtr()->getFramePtr()->isKey() ? "  KF" : "  F")
                                << pt->getOriginPtr()->getFramePtr()->id() << endl;
                            if (pt->getLastPtr())
                                cout << "      l: C" << pt->getLastPtr()->id() << " - " << (pt->getLastPtr()->getFramePtr()->isKey() ? "  KF" : "  F")
                                << pt->getLastPtr()->getFramePtr()->id() << endl;
                            if (pt->getIncomingPtr())
                                cout << "      i: C" << pt->getIncomingPtr()->id() << endl;
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
                cout << (F->isFixed() ? "    Fix" : "    Est") << ", ts=" << std::setprecision(5)
                        << F->getTimeStamp().get();
                cout << ",\t x = ( " << std::setprecision(2) << F->getState().transpose() << " )";
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
                    cout << "    C" << (C->isMotion() ? "M" : "") << C->id() << " " << C->getType();
                    
                    if(C->getSensorPtr() != nullptr)
                    {
                        cout << " -> S" << C->getSensorPtr()->id();
                        cout << (C->getSensorPtr()->isExtrinsicDynamic() ? " [Dyn, ": " [Sta, ");
                        cout << (C->getSensorPtr()->isIntrinsicDynamic() ? "Dyn]" : "Sta]");
                    }
                    else
                        cout << " -> S-";

                    cout << ((depth < 3) ? " -- " + std::to_string(C->getFeatureList().size()) + "f" : "");
                    if (constr_by)
                    {
                        cout << "  <-- ";
                        for (auto cby : C->getConstrainedByList())
                            cout << "c" << cby->id() << " \t";
                    }
                    cout << endl;

                    if (state_blocks)
                        for(auto sb : C->getStateBlockVec())
                            if(sb != nullptr)
                            {
                                cout << "      sb: ";
                                cout << (sb->isFixed() ? "Fix" : "Est");
                                if (metric)
                                    cout << std::setprecision(3) << " (" << sb->getState().transpose() << " )";
                                cout << endl;
                            }

                    if (C->isMotion() && metric)
                    {
                        try
                        {
                            CaptureMotionPtr CM = std::static_pointer_cast<CaptureMotion>(C);
                            cout << "      buffer size  :  " << CM->getBuffer().get().size() << endl;
                            if ( CM->getCalibSize() > 0 && ! CM->getBuffer().get().empty())
                            {
                                cout << "      delta preint : (" << CM->getDeltaPreint().transpose() << ")" << endl;
                                cout << "      calib preint : (" << CM->getCalibrationPreint().transpose() << ")" << endl;
                                cout << "      jacob preint : (" << CM->getJacobianCalib().row(0) << ")" << endl;
                                cout << "      calib current: (" << CM->getCalibration().transpose() << ")" << endl;
                                cout << "      delta correct: (" << CM->getDeltaCorrected(CM->getCalibration()).transpose() << ")" << endl;
                            }
                        }
                        catch  (std::runtime_error& e)
                        {
                        }
                    }

                    if (depth >= 3)
                    {
                        // Features
                        for (auto f : C->getFeatureList())
                        {
                            cout << "      f" << f->id() << " trk" << f->trackId() << " " << f->getType() << ((depth < 4) ? " -- " + std::to_string(f->getConstraintList().size()) + "c  " : "");
                            if (constr_by)
                            {
                                cout << "  <--\t";
                                for (auto cby : f->getConstrainedByList())
                                    cout << "c" << cby->id() << " \t";
                            }
                            cout << endl;
                            if (metric)
                                cout << "        m = ( " << std::setprecision(3) << f->getMeasurement().transpose()
                                        << " )" << endl;
                            if (depth >= 4)
                            {
                                // Constraints
                                for (auto c : f->getConstraintList())
                                {
                                    cout << "        c" << c->id() << " " << c->getType() << " -->";
                                    if (c->getFrameOtherPtr() == nullptr && c->getCaptureOtherPtr() == nullptr && c->getFeatureOtherPtr() == nullptr && c->getLandmarkOtherPtr() == nullptr)
                                        cout << " A";
                                    if (c->getFrameOtherPtr() != nullptr)
                                        cout << " F" << c->getFrameOtherPtr()->id();
                                    if (c->getCaptureOtherPtr() != nullptr)
                                        cout << " C" << c->getCaptureOtherPtr()->id();
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
            cout << "  L" << L->id() << " " << L->getType();
            if (constr_by)
            {
                cout << "\t<-- ";
                for (auto cby : L->getConstrainedByList())
                    cout << "c" << cby->id() << " \t";
            }
            cout << endl;
            if (metric)
            {
                cout << (L->isFixed() ? "    Fix" : "    Est");
                cout << ",\t x = ( " << std::setprecision(2) << L->getState().transpose() << " )";
                cout << endl;
            }
            if (state_blocks)
            {
                cout << "    sb:";
                for (auto sb : L->getStateBlockVec())
                    if (sb != nullptr)
                        cout << (sb->isFixed() ? " Fix" : " Est");
                cout << endl;
            }
        } // for L
    }
    cout << "-----------------------------------------" << endl;
    cout << endl;
}

FrameBasePtr wolf::Problem::closestKeyFrameToTimeStamp(const TimeStamp& _ts)
{
    return trajectory_ptr_->closestKeyFrameToTimeStamp(_ts);
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
                for (auto sb : C->getStateBlockVec())
                {
                    cout <<  "      sb @ " << sb.get();
                    if (sb)
                    {
                        auto lp = sb->getLocalParametrizationPtr();
                        if (lp)
                            cout <<  " (lp @ " << lp.get() << ")";
                    }
                    cout << endl;
                }
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
                    auto Co = c->getCaptureOtherPtr();
                    auto fo = c->getFeatureOtherPtr();
                    auto Lo = c->getLandmarkOtherPtr();

                    if ( !Fo && !Co && !fo && !Lo )    // case ABSOLUTE:
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

                    // find constrained_by pointer in constrained capture
                    if ( Co )  // case CAPTURE:
                    {
                        if (verbose_level > 0)
                            cout << " --> C" << Co->id() << " <- ";
                        bool found = false;
                        for (auto cby : Co->getConstrainedByList())
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
                        // find in own Capture
                        found = found || (std::find(C->getStateBlockVec().begin(), C->getStateBlockVec().end(), sb) != C->getStateBlockVec().end());
                        // find in own Sensor
                        if (S)
                            found = found || (std::find(S->getStateBlockVec().begin(), S->getStateBlockVec().end(), sb) != S->getStateBlockVec().end());
                        // find in constrained Frame
                        if (Fo)
                            found = found || (std::find(Fo->getStateBlockVec().begin(), Fo->getStateBlockVec().end(), sb) != Fo->getStateBlockVec().end());
                        // find in constrained Capture
                        if (Co)
                            found = found || (std::find(Co->getStateBlockVec().begin(), Co->getStateBlockVec().end(), sb) != Co->getStateBlockVec().end());
                        // find in constrained Feature
                        if (fo)
                        {
                            // find in constrained feature's Frame
                            FrameBasePtr foF = fo->getFramePtr();
                            found = found || (std::find(foF->getStateBlockVec().begin(), foF->getStateBlockVec().end(), sb) != foF->getStateBlockVec().end());
                            // find in constrained feature's Capture
                            CaptureBasePtr foC = fo->getCapturePtr();
                            found = found || (std::find(foC->getStateBlockVec().begin(), foC->getStateBlockVec().end(), sb) != foC->getStateBlockVec().end());
                            // find in constrained feature's Sensor
                            SensorBasePtr foS = fo->getCapturePtr()->getSensorPtr();
                            found = found || (std::find(foS->getStateBlockVec().begin(), foS->getStateBlockVec().end(), sb) != foS->getStateBlockVec().end());
                        }
                        // find in constrained landmark
                        if (Lo)
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

void Problem::print(const std::string& depth, bool constr_by, bool metric, bool state_blocks)
{
    if (depth.compare("T") == 0)
        print(0, constr_by, metric, state_blocks);
    else if (depth.compare("F") == 0)
        print(1, constr_by, metric, state_blocks);
    else if (depth.compare("C") == 0)
        print(2, constr_by, metric, state_blocks);
    else if (depth.compare("f") == 0)
        print(3, constr_by, metric, state_blocks);
    else if (depth.compare("c") == 0)
        print(4, constr_by, metric, state_blocks);
    else
        print(0, constr_by, metric, state_blocks);
}

} // namespace wolf
