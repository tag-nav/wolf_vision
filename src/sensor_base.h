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
        SizeEigen calib_size_;

        static unsigned int sensor_id_count_; ///< Object counter (acts as simple ID factory)

    protected:
        unsigned int sensor_id_;   // sensor ID

        bool extrinsic_dynamic_;// extrinsic parameters vary with time? If so, they will be taken from the Capture nodes.
        bool intrinsic_dynamic_;// intrinsic parameters vary with time? If so, they will be taken from the Capture nodes.
        bool has_capture_;      // indicates this sensor took at least one capture

        Eigen::VectorXs noise_std_; // std of sensor noise
        Eigen::MatrixXs noise_cov_; // cov matrix of noise

        std::map<unsigned int,FeatureBasePtr> params_prior_map_; // Priors (value and covariance) of extrinsic & intrinsic state blocks (by index in state_block_vec_)

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

        unsigned int id();

        virtual void setProblem(ProblemPtr _problem) final;

        HardwareBasePtr getHardwarePtr();
        void setHardwarePtr(const HardwareBasePtr _hw_ptr);

        ProcessorBasePtr addProcessor(ProcessorBasePtr _proc_ptr);
        ProcessorBaseList& getProcessorList();

        CaptureBasePtr lastKeyCapture(void);
        CaptureBasePtr lastCapture(const TimeStamp& _ts);

        bool process(const CaptureBasePtr capture_ptr);

        // State blocks
        const std::vector<StateBlockPtr>& getStateBlockVec() const;
        std::vector<StateBlockPtr>& getStateBlockVec();
        StateBlockPtr getStateBlockPtrStatic(unsigned int _i) const;
        StateBlockPtr getStateBlockPtr(unsigned int _i);
        StateBlockPtr getStateBlockPtr(unsigned int _i, const TimeStamp& _ts);
        void setStateBlockPtrStatic(unsigned int _i, const StateBlockPtr _sb_ptr);
        void resizeStateBlockVec(unsigned int _size);

        bool isStateBlockDynamic(unsigned int _i, const TimeStamp& _ts, CaptureBasePtr& cap);
        bool isStateBlockDynamic(unsigned int _i, CaptureBasePtr& cap);
        bool isStateBlockDynamic(unsigned int _i, const TimeStamp& _ts);
        bool isStateBlockDynamic(unsigned int _i);

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

        /** \brief Add an absolute constraint to a parameter
         *
         * Add an absolute constraint to a parameter
         * \param _i state block index (in state_block_vec_) of the parameter to be constrained
         * \param _x prior value
         * \param _cov covariance
         * \param _start_idx state segment starting index (not used in quaternions)
         * \param _size state segment size (-1: whole state) (not used in quaternions)
         *
         **/
        void addPriorParameter(const unsigned int _i,
                               const Eigen::VectorXs& _x,
                               const Eigen::MatrixXs& _cov,
                               unsigned int _start_idx = 0,
                               int _size = -1);
        void addPriorP(const Eigen::VectorXs& _x,
                       const Eigen::MatrixXs& _cov,
                       unsigned int _start_idx = 0,
                       int _size = -1);
        void addPriorO(const Eigen::VectorXs& _x,
                       const Eigen::MatrixXs& _cov);
        void addPriorIntrinsics(const Eigen::VectorXs& _x,
                                const Eigen::MatrixXs& _cov,
                                unsigned int _start_idx = 0,
                                int _size = -1);

        SizeEigen getCalibSize() const;
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
        void setExtrinsicDynamic(bool _extrinsic_dynamic);
        void setIntrinsicDynamic(bool _intrinsic_dynamic);

    protected:
        SizeEigen computeCalibSize() const;

    private:
        void updateCalibSize();
};

}

#include "problem.h"
#include "hardware_base.h"
#include "capture_base.h"
#include "processor_base.h"

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
    assert (_i < state_block_vec_.size() && "Setting a state block pointer out of the vector range!");
    assert((params_prior_map_.find(_i) == params_prior_map_.end() || _sb_ptr == nullptr) && "overwriting a state block that has an absolute constraint");
    state_block_vec_[_i] = _sb_ptr;
}

inline void SensorBase::resizeStateBlockVec(unsigned int _size)
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

inline void SensorBase::addPriorP(const Eigen::VectorXs& _x, const Eigen::MatrixXs& _cov, unsigned int _start_idx, int _size)
{
    addPriorParameter(0, _x, _cov, _start_idx, _size);
}

inline void SensorBase::addPriorO(const Eigen::VectorXs& _x, const Eigen::MatrixXs& _cov)
{
    addPriorParameter(1, _x, _cov);
}

inline void SensorBase::addPriorIntrinsics(const Eigen::VectorXs& _x, const Eigen::MatrixXs& _cov, unsigned int _start_idx, int _size)
{
    addPriorParameter(2, _x, _cov);
}

inline SizeEigen SensorBase::getCalibSize() const
{
    return calib_size_;
}

inline void SensorBase::updateCalibSize()
{
    calib_size_ = computeCalibSize();
}

inline void SensorBase::setExtrinsicDynamic(bool _extrinsic_dynamic)
{
    extrinsic_dynamic_ = _extrinsic_dynamic;
}

inline void SensorBase::setIntrinsicDynamic(bool _intrinsic_dynamic)
{
    intrinsic_dynamic_ = _intrinsic_dynamic;
}



} // namespace wolf

#endif
