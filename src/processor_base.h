#ifndef PROCESSOR_BASE_H_
#define PROCESSOR_BASE_H_

// Fwd refs
namespace wolf{
class SensorBase;
}

// Wolf includes
#include "wolf.h"
#include "node_base.h"
#include "time_stamp.h"

// std
#include <memory>
#include <map>

namespace wolf {

/** \brief Key frame class pack
 *
 * To store a key_frame with an associated time tolerance.
 */
class KFPack
{
    public:
        KFPack(const FrameBasePtr _key_frame, const Scalar _time_tolerance) : key_frame(_key_frame), time_tolerance(_time_tolerance) {};
        ~KFPack(){};
        FrameBasePtr key_frame;
        Scalar time_tolerance;
};

WOLF_PTR_TYPEDEFS(KFPack);



/** \brief Buffer of Key frame class objects
 *
 * Object and functions to manage a buffer of KFPack objects.
 */
class KFPackBuffer
{
    public:

        typedef std::map<TimeStamp,KFPackPtr>::iterator Iterator; // buffer iterator

        KFPackBuffer(void);
        ~KFPackBuffer(void);

        /**\brief Select a Pack from the buffer
         *
         *  Select from the buffer the closest pack (w.r.t. time stamp),
         * respecting a defined time tolerance
         */
        KFPackPtr selectPack(const TimeStamp& _time_stamp, const Scalar& _time_tolerance);

        /**\brief Buffer size
         *
         */
        size_t size(void);

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

        std::map<TimeStamp,KFPackPtr> container_; // Main buffer container
};

/** \brief base struct for processor parameters
 *
 * Derive from this struct to create structs of processor parameters.
 */
struct ProcessorParamsBase
{
    ProcessorParamsBase()          {time_tolerance = 0;};
    virtual ~ProcessorParamsBase() = default;

    std::string type;
    std::string name;
    Scalar time_tolerance;
};

//class ProcessorBase
class ProcessorBase : public NodeBase, public std::enable_shared_from_this<ProcessorBase>
{
    protected:
        unsigned int processor_id_;
        Scalar time_tolerance_;         ///< self time tolerance for adding a capture into a frame
        KFPackBuffer kf_pack_buffer_;

    private:
        SensorBaseWPtr sensor_ptr_;

        bool is_removing_; ///< A flag for safely removing nodes from the Wolf tree. See remove().
        static unsigned int processor_id_count_;

    public:
        ProcessorBase(const std::string& _type, const Scalar& _time_tolerance = 0);
        virtual ~ProcessorBase();
        void remove();

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

        virtual bool permittedKeyFrame() final;

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

//        virtual bool keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tolerance) = 0;

        void keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tol_other);

        SensorBasePtr getSensorPtr();
        const SensorBasePtr getSensorPtr() const;
        void setSensorPtr(SensorBasePtr _sen_ptr){sensor_ptr_ = _sen_ptr;}

        virtual bool isMotion();

        void setTimeTolerance(Scalar _time_tolerance);

};

}

#include "sensor_base.h"
#include "constraint_base.h"

namespace wolf {

inline bool ProcessorBase::isMotion()
{
    return false;
}

inline unsigned int ProcessorBase::id()
{
    return processor_id_;
}

inline SensorBasePtr ProcessorBase::getSensorPtr()
{
    return sensor_ptr_.lock();
}

inline const SensorBasePtr ProcessorBase::getSensorPtr() const
{
    return sensor_ptr_.lock();
}

inline void ProcessorBase::setTimeTolerance(Scalar _time_tolerance)
{
    time_tolerance_ = _time_tolerance;
}

inline KFPackBuffer::KFPackBuffer(void)
{

}

inline KFPackBuffer::~KFPackBuffer(void)
{

}

inline void KFPackBuffer::clear()
{
    container_.clear();
}

inline bool KFPackBuffer::empty()
{
    return container_.empty();
}

inline size_t KFPackBuffer::size(void)
{
    return container_.size();
}

} // namespace wolf

#endif
