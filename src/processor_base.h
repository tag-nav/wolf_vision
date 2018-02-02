#ifndef PROCESSOR_BASE_H_
#define PROCESSOR_BASE_H_

// Fwd refs
namespace wolf{
class SensorBase;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"

// std
#include <memory>

namespace wolf {



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

        virtual bool keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tolerance) = 0;

        SensorBasePtr getSensorPtr();
        const SensorBasePtr getSensorPtr() const;
        void setSensorPtr(SensorBasePtr _sen_ptr){sensor_ptr_ = _sen_ptr;}

        virtual bool isMotion();

        void setTimeTolerance(Scalar _time_tolerance);

    protected:
        unsigned int processor_id_;
        Scalar time_tolerance_;         ///< self time tolerance for adding a capture into a frame
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

} // namespace wolf

#endif
