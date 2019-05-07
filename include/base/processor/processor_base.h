#ifndef PROCESSOR_BASE_H_
#define PROCESSOR_BASE_H_

// Fwd refs
namespace wolf{
class SensorBase;
}

// Wolf includes
#include "base/common/wolf.h"
#include "base/common/node_base.h"
#include "base/common/time_stamp.h"
#include "base/frame/frame_base.h"

// std
#include <memory>
#include <map>

namespace wolf {

/** \brief Key frame class pack
 *
 * To store a key_frame with an associated time tolerance.
 *
 * Used in keyframe callbacks as the minimal pack of information needed by the processor receiving the callback.
 */
class PackKeyFrame
{
    public:
        PackKeyFrame(const FrameBasePtr _key_frame, const Scalar _time_tolerance) : key_frame(_key_frame), time_tolerance(_time_tolerance) {};
        ~PackKeyFrame(){};
        FrameBasePtr key_frame;
        Scalar time_tolerance;
};

WOLF_PTR_TYPEDEFS(PackKeyFrame);

/** \brief Buffer of Key frame class objects
 *
 * Object and functions to manage a buffer of KFPack objects.
 */
class PackKeyFrameBuffer
{
    public:

        typedef std::map<TimeStamp,PackKeyFramePtr>::iterator Iterator; // buffer iterator

        PackKeyFrameBuffer(void);
        ~PackKeyFrameBuffer(void);

        /**\brief Select a Pack from the buffer
         *
         *  Select from the buffer the closest pack (w.r.t. time stamp),
         * respecting a defined time tolerances
         */
        PackKeyFramePtr selectPack(const TimeStamp& _time_stamp, const Scalar& _time_tolerance);
        PackKeyFramePtr selectPack(const CaptureBasePtr _capture, const Scalar& _time_tolerance);

        PackKeyFramePtr selectFirstPackBefore(const TimeStamp& _time_stamp, const Scalar& _time_tolerance);
        PackKeyFramePtr selectFirstPackBefore(const CaptureBasePtr _capture, const Scalar& _time_tolerance);

        /**\brief Buffer size
         *
         */
        SizeStd size(void);

        /**\brief Add a pack to the buffer
         *
         */
        void add(const FrameBasePtr& _key_frame, const Scalar& _time_tolerance);

        /**\brief Remove all packs in the buffer with a time stamp older than the specified
         *
         */
        void removeUpTo(const TimeStamp& _time_stamp);

        /**\brief Check time tolerance
         *
         * Check if the time distance between two time stamps is smaller than
         * the minimum time tolerance of the two frames.
         */
        bool checkTimeTolerance(const TimeStamp& _time_stamp1, const Scalar& _time_tolerance1, const TimeStamp& _time_stamp2, const Scalar& _time_tolerance2);

        /**\brief Clear the buffer
         *
         */
        void clear();

        /**\brief Empty the buffer
         *
         */
        bool empty();

        /**\brief Print buffer information
         *
         */
        void print();

    private:

        std::map<TimeStamp,PackKeyFramePtr> container_; // Main buffer container
};

/** \brief base struct for processor parameters
 *
 * Derive from this struct to create structs of processor parameters.
 */
struct ProcessorParamsBase
{
    ProcessorParamsBase() = default;

    ProcessorParamsBase(bool _voting_active,
                        Scalar _time_tolerance,
                        bool _voting_aux_active = false) :
        voting_active(_voting_active),
        voting_aux_active(_voting_aux_active),
        time_tolerance(_time_tolerance)
    {
      //
    }

    virtual ~ProcessorParamsBase() = default;

    bool voting_active = false;     ///< Whether this processor is allowed to vote for a Key Frame or not
    bool voting_aux_active = false; ///< Whether this processor is allowed to vote for an Auxiliary Frame or not

    ///< maximum time difference between a Keyframe time stamp and
    /// a particular Capture of this processor to allow assigning
    /// this Capture to the Keyframe.
    Scalar time_tolerance = Scalar(0);
};

//class ProcessorBase
class ProcessorBase : public NodeBase, public std::enable_shared_from_this<ProcessorBase>
{
    protected:
        unsigned int processor_id_;
        ProcessorParamsBasePtr params_;
        PackKeyFrameBuffer kf_pack_buffer_;

    private:
        SensorBaseWPtr sensor_ptr_;

        static unsigned int processor_id_count_;

    public:
        ProcessorBase(const std::string& _type, ProcessorParamsBasePtr _params);
        virtual ~ProcessorBase();
        virtual void configure(SensorBasePtr _sensor) = 0;
        virtual void remove();

        unsigned int id();

        virtual void process(CaptureBasePtr _capture_ptr) = 0;

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame() = 0;

        /** \brief Vote for Auxiliary Frame generation
         *
         * If a Auxiliary Frame criterion is validated, this function returns true,
         * meaning that it wants to create a Auxiliary Frame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create Auxiliary Frames!
         */
        virtual bool voteForAuxFrame(){return false;};

        virtual bool permittedKeyFrame() final;

        virtual bool permittedAuxFrame() final;

        /**\brief make a Frame with the provided Capture
         *
         * Provide the following functionality:
         *   - Construct a Frame,
         *   - Put it in the Trajectory, and
         *   - Add the provided capture on it.
         */
        FrameBasePtr emplaceFrame(FrameType _type, CaptureBasePtr _capture_ptr);

        /**\brief make a Frame with the provided Capture
         *
         * Provide the following functionality:
         *   - Construct a Frame,
         *   - Set its state vector
         *   - Put it in the Trajectory, and
         *   - Add the provided capture on it.
         */
        FrameBasePtr emplaceFrame(FrameType _type, CaptureBasePtr _capture_ptr, const Eigen::VectorXs& _state);

        virtual void keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tol_other);

        SensorBasePtr getSensor();
        const SensorBasePtr getSensor() const;
        void setSensor(SensorBasePtr _sen_ptr){sensor_ptr_ = _sen_ptr;}

        virtual bool isMotion();

        void setTimeTolerance(Scalar _time_tolerance);

        bool isVotingActive() const;

        bool isVotingAuxActive() const;

        void setVotingActive(bool _voting_active = true);

        void link(SensorBasePtr);
        template<typename classType, typename... T>
        static std::shared_ptr<ProcessorBase> emplace(SensorBasePtr _sen_ptr, T&&... all);
        void setVotingAuxActive(bool _voting_active = true);
};

inline bool ProcessorBase::isVotingActive() const
{
    return params_->voting_active;
}

inline bool ProcessorBase::isVotingAuxActive() const
{
    return params_->voting_aux_active;
}

inline void ProcessorBase::setVotingActive(bool _voting_active)
{
    params_->voting_active = _voting_active;
}

inline void ProcessorBase::setVotingAuxActive(bool _voting_active)
{
    params_->voting_aux_active = _voting_active;
}

}

#include "base/sensor/sensor_base.h"
#include "base/factor/factor_base.h"

namespace wolf {

template<typename classType, typename... T>
std::shared_ptr<ProcessorBase> ProcessorBase::emplace(SensorBasePtr _sen_ptr, T&&... all)
{
    ProcessorBasePtr prc = std::make_shared<classType>(std::forward<T>(all)...);
    prc->link(_sen_ptr);
    return prc;
}

inline bool ProcessorBase::isMotion()
{
    return false;
}

inline unsigned int ProcessorBase::id()
{
    return processor_id_;
}

inline SensorBasePtr ProcessorBase::getSensor()
{
    return sensor_ptr_.lock();
}

inline const SensorBasePtr ProcessorBase::getSensor() const
{
    return sensor_ptr_.lock();
}

inline void ProcessorBase::setTimeTolerance(Scalar _time_tolerance)
{
    params_->time_tolerance = _time_tolerance;
}

inline PackKeyFrameBuffer::PackKeyFrameBuffer(void)
{

}

inline PackKeyFrameBuffer::~PackKeyFrameBuffer(void)
{

}

inline void PackKeyFrameBuffer::clear()
{
    container_.clear();
}

inline bool PackKeyFrameBuffer::empty()
{
    return container_.empty();
}

inline SizeStd PackKeyFrameBuffer::size(void)
{
    return container_.size();
}

} // namespace wolf

#endif
