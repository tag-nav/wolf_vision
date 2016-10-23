#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_

// Fwd refs
namespace wolf{
class HardwareBase;
class ProcessorBase;
class StateBlock;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"

//std includes

namespace wolf {


/** \brief base struct for intrinsic sensor parameters
 *
 * Derive from this struct to create structs of sensor intrinsic parameters.
 */
struct IntrinsicsBase
{
        std::string type;
        std::string name;
};

class SensorBase : public NodeBase, public std::enable_shared_from_this<SensorBase>
{
    private:
        HardwareBaseWPtr hardware_ptr_;
        ProcessorBaseList processor_list_;
        std::vector<StateBlockPtr> state_block_vec_; ///< vector of state blocks, in the order P, O, intrinsic.

        static unsigned int sensor_id_count_; ///< Object counter (acts as simple ID factory)

    protected:
        unsigned int sensor_id_;   // sensor ID
        SensorType type_id_;       // the type of sensor. See wolf.h for a list of all sensor types.

        bool extrinsic_dynamic_;// extrinsic parameters vary with time? If so, they will be taken from the Capture nodes. TODO: Not Yet Implemented.

        Eigen::VectorXs noise_std_; // std of sensor noise
        Eigen::MatrixXs noise_cov_; // cov matrix of noise

    public:
        /** \brief Constructor with noise size
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _intr_ptr StateBlock pointer to the sensor intrinsic params that might be estimated (if unfixed).
         * \param _noise_size dimension of the noise term
         * \param _extr_dyn Flag indicating if extrinsics are dynamic (moving) or static (not moving)
         *
         **/
        SensorBase(const SensorType & _tp, const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _intr_ptr, const unsigned int _noise_size, const bool _extr_dyn = false);

        /** \brief Constructor with noise std vector
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _intr_ptr StateBlock pointer to the sensor intrinsic params that might be estimated (if unfixed).
         * \param _noise_std standard deviations of the noise term
         * \param _extr_dyn Flag indicating if extrinsics are dynamic (moving) or static (not moving)
         *
         **/
        SensorBase(const SensorType & _tp, const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _intr_ptr, const Eigen::VectorXs & _noise_std, const bool _extr_dyn = false);
        virtual ~SensorBase();
        void remove();

        unsigned int id();
        SensorType typeId();

        // State blocks
        const std::vector<StateBlockPtr>& getStateBlockVec() const
        {
            return state_block_vec_;
        }
        std::vector<StateBlockPtr>& getStateBlockVec()
        {
            return state_block_vec_;
        }
        StateBlockPtr getStateBlockPtr(unsigned int _i) const
        {
            assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
            return state_block_vec_[_i];
        }
        void setStateBlockPtr(unsigned int _i, StateBlockPtr _sb_ptr)
        {
            state_block_vec_[_i] = _sb_ptr;
        }

        StateBlockPtr getPPtr() const;
        StateBlockPtr getOPtr() const;
        /** \brief intrinsic parameters.
         * Use it if desired. By using this StateBlock, WOLF will be able to auto-calibrate these parameters.
         * To do so, just unfix() it. After the calibration process, you can fix() it again if desired.
         *
         * (Note: Many other intrinsic parameters can be stored as members of the classes derived from this.
         * We recommend you use a struct for this purpose if the number of intrinsic parameters is large.)
         */
        StateBlockPtr getIntrinsicPtr() const;
        void setPPtr(StateBlockPtr _p_ptr);
        void setOPtr(StateBlockPtr _o_ptr);
        void setIntrinsicPtr(StateBlockPtr _intr_ptr);
        void removeStateBlocks();

        ProcessorBasePtr addProcessor(ProcessorBasePtr _proc_ptr);
        ProcessorBaseList& getProcessorList();

        void fix();
        void unfix();

        /** \brief Adds all stateBlocks of the sensor to the wolfProblem list of new stateBlocks
         **/
        virtual void registerNewStateBlocks();

        /**
         * Check if sensor is dynamic
         */
        bool isExtrinsicDynamic();

        void setNoise(const Eigen::VectorXs & _noise_std);
        Eigen::VectorXs getNoiseStd();
        Eigen::MatrixXs getNoiseCov();

        ProblemPtr getProblem();
        HardwareBasePtr getHardwarePtr();
        void setHardwarePtr(HardwareBasePtr _hw_ptr);

};

}

#include "processor_base.h"
#include "hardware_base.h"

namespace wolf{

inline wolf::ProblemPtr SensorBase::getProblem()
{
    ProblemPtr prb = problem_ptr_.lock();
    if (!prb){
        HardwareBasePtr hw = hardware_ptr_.lock();
    if (hw)
    {
        prb = hw->getProblem();
        problem_ptr_ = prb;
    }
    }
    return prb;
}

inline unsigned int SensorBase::id()
{
    return sensor_id_;
}

inline wolf::SensorType SensorBase::typeId()
{
    return type_id_;
}

inline ProcessorBasePtr SensorBase::addProcessor(ProcessorBasePtr _proc_ptr)
{
    processor_list_.push_back(_proc_ptr);
    _proc_ptr->setSensorPtr(shared_from_this());
    _proc_ptr->setProblem(getProblem());
    return _proc_ptr;
}

inline ProcessorBaseList& SensorBase::getProcessorList()
{
    return processor_list_;
}

inline StateBlockPtr SensorBase::getPPtr() const
{
    return getStateBlockPtr(0);
}

inline StateBlockPtr SensorBase::getOPtr() const
{
    return getStateBlockPtr(1);
}

inline StateBlockPtr SensorBase::getIntrinsicPtr() const
{
    return getStateBlockPtr(2);
}

inline bool SensorBase::isExtrinsicDynamic()
{
    return extrinsic_dynamic_;
}

inline Eigen::VectorXs SensorBase::getNoiseStd()
{
    return noise_std_;
}

inline Eigen::MatrixXs SensorBase::getNoiseCov()
{
    return noise_cov_;
}

inline HardwareBasePtr SensorBase::getHardwarePtr()
{
    return hardware_ptr_.lock();
}

inline void SensorBase::setPPtr(StateBlockPtr _p_ptr)
{
    setStateBlockPtr(0, _p_ptr);
}

inline void SensorBase::setOPtr(StateBlockPtr _o_ptr)
{
    setStateBlockPtr(1, _o_ptr);
}

inline void SensorBase::setIntrinsicPtr(StateBlockPtr _intr_ptr)
{
    setStateBlockPtr(2, _intr_ptr);
}

inline void SensorBase::setHardwarePtr(HardwareBasePtr _hw_ptr)
{
    hardware_ptr_ = _hw_ptr;
}

} // namespace wolf

#endif
