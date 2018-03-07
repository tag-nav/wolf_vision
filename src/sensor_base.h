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
#include "time_stamp.h"

//std includes

namespace wolf {


/** \brief base struct for intrinsic sensor parameters
 *
 * Derive from this struct to create structs of sensor intrinsic parameters.
 */
struct IntrinsicsBase
{
  IntrinsicsBase()          = default;
  virtual ~IntrinsicsBase() = default;

  std::string type;
  std::string name;
};

class SensorBase : public NodeBase, public std::enable_shared_from_this<SensorBase>
{
    private:
        HardwareBaseWPtr hardware_ptr_;
        ProcessorBaseList processor_list_;
        std::vector<StateBlockPtr> state_block_vec_; ///< vector of state blocks, in the order P, O, intrinsic.
        Size calib_size_;

        static unsigned int sensor_id_count_; ///< Object counter (acts as simple ID factory)
        bool is_removing_; ///< A flag for safely removing nodes from the Wolf tree. See remove().

    protected:
        unsigned int sensor_id_;   // sensor ID

        bool extrinsic_dynamic_;// extrinsic parameters vary with time? If so, they will be taken from the Capture nodes.
        bool intrinsic_dynamic_;// intrinsic parameters vary with time? If so, they will be taken from the Capture nodes.
        bool has_capture_;      // indicates this sensor took at least one capture

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
        SensorBase(const std::string& _type,
                   StateBlockPtr _p_ptr,
                   StateBlockPtr _o_ptr,
                   StateBlockPtr _intr_ptr,
                   const unsigned int _noise_size,
                   const bool _extr_dyn = false,
                   const bool _intr_dyn = false);

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
        SensorBase(const std::string& _type,
                   StateBlockPtr _p_ptr,
                   StateBlockPtr _o_ptr,
                   StateBlockPtr _intr_ptr,
                   const Eigen::VectorXs & _noise_std,
                   const bool _extr_dyn = false,
                   const bool _intr_dyn = false);

        virtual ~SensorBase();
        void remove();

        unsigned int id();

        HardwareBasePtr getHardwarePtr();
        void setHardwarePtr(const HardwareBasePtr _hw_ptr);

        ProcessorBasePtr addProcessor(ProcessorBasePtr _proc_ptr);
        ProcessorBaseList& getProcessorList();

        CaptureBasePtr lastCapture(void);
        CaptureBasePtr lastCapture(const TimeStamp& _ts);

        bool process(const CaptureBasePtr capture_ptr);

        // State blocks
        const std::vector<StateBlockPtr>& getStateBlockVec() const;
        std::vector<StateBlockPtr>& getStateBlockVec();
        StateBlockPtr getStateBlockPtrStatic(unsigned int _i) const;
        StateBlockPtr getStateBlockPtrDynamic(unsigned int _i);
        StateBlockPtr getStateBlockPtrDynamic(unsigned int _i, const TimeStamp& _ts);
        void setStateBlockPtrStatic(unsigned int _i, const StateBlockPtr _sb_ptr);
        void resizeStateBlockVec(int _size);

        StateBlockPtr getPPtr(const TimeStamp _ts);
        StateBlockPtr getOPtr(const TimeStamp _ts);
        StateBlockPtr getIntrinsicPtr(const TimeStamp _ts);
        StateBlockPtr getPPtr() ;
        StateBlockPtr getOPtr();
        StateBlockPtr getIntrinsicPtr();
        void setPPtr(const StateBlockPtr _p_ptr);
        void setOPtr(const StateBlockPtr _o_ptr);
        void setIntrinsicPtr(const StateBlockPtr _intr_ptr);
        void removeStateBlocks();

        void fix();
        void unfix();
        void fixExtrinsics();
        void unfixExtrinsics();
        void fixIntrinsics();
        void unfixIntrinsics();

        Size getCalibSize() const;
        Eigen::VectorXs getCalibration() const;

        virtual void registerNewStateBlocks();

        bool isExtrinsicDynamic() const;
        bool isIntrinsicDynamic() const;
        bool hasCapture() const {return has_capture_;}
        void setHasCapture() {has_capture_ = true;}
        bool extrinsicsInCaptures() const { return extrinsic_dynamic_ && has_capture_; }
        bool intrinsicsInCaptures() const { return intrinsic_dynamic_ && has_capture_; }

        void setNoiseStd(const Eigen::VectorXs & _noise_std);
        void setNoiseCov(const Eigen::MatrixXs & _noise_std);
        Eigen::VectorXs getNoiseStd();
        Eigen::MatrixXs getNoiseCov();

    protected:
        Size computeCalibSize() const;

    private:
        void updateCalibSize();
};

}

#include "problem.h"
#include "hardware_base.h"
#include "processor_base.h"
#include "capture_base.h"

namespace wolf{

inline unsigned int SensorBase::id()
{
    return sensor_id_;
}

inline ProcessorBaseList& SensorBase::getProcessorList()
{
    return processor_list_;
}

inline const std::vector<StateBlockPtr>& SensorBase::getStateBlockVec() const
{
    return state_block_vec_;
}

inline std::vector<StateBlockPtr>& SensorBase::getStateBlockVec()
{
    return state_block_vec_;
}

inline StateBlockPtr SensorBase::getStateBlockPtrStatic(unsigned int _i) const
{
    assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
    return state_block_vec_[_i];
}

inline void SensorBase::setStateBlockPtrStatic(unsigned int _i, const StateBlockPtr _sb_ptr)
{
    state_block_vec_[_i] = _sb_ptr;
}

inline void SensorBase::resizeStateBlockVec(int _size)
{
    if (_size > state_block_vec_.size())
        state_block_vec_.resize(_size);
}

inline bool SensorBase::isExtrinsicDynamic() const
{
    // If this Sensor has no Capture yet, we'll consider it static
    return extrinsic_dynamic_;
}

inline bool SensorBase::isIntrinsicDynamic() const
{
    // If this Sensor has no Capture yet, we'll consider it static
    return intrinsic_dynamic_;
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

inline void SensorBase::setPPtr(const StateBlockPtr _p_ptr)
{
    setStateBlockPtrStatic(0, _p_ptr);
}

inline void SensorBase::setOPtr(const StateBlockPtr _o_ptr)
{
    setStateBlockPtrStatic(1, _o_ptr);
}

inline void SensorBase::setIntrinsicPtr(const StateBlockPtr _intr_ptr)
{
    setStateBlockPtrStatic(2, _intr_ptr);
}

inline void SensorBase::setHardwarePtr(const HardwareBasePtr _hw_ptr)
{
    hardware_ptr_ = _hw_ptr;
}

inline wolf::Size SensorBase::getCalibSize() const
{
    return calib_size_;
}

inline void SensorBase::updateCalibSize()
{
    calib_size_ = computeCalibSize();
}



} // namespace wolf

#endif
