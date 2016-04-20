#ifndef PROCESSOR_BASE_H_
#define PROCESSOR_BASE_H_

// Fwd refs
namespace wolf{
class SensorBase;
class NodeTerminus;
}

//Wolf includes
#include "wolf.h"
#include "node_linked.h"

namespace wolf {

//class ProcessorBase
class ProcessorBase : public NodeLinked<SensorBase, NodeTerminus>
{
    public:
        ProcessorBase(ProcessorType _tp);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ProcessorBase();

        virtual void process(CaptureBase* _capture_ptr) = 0;

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame() = 0;

        virtual bool permittedKeyFrame() final;

        virtual bool keyFrameCallback(FrameBase* _keyframe_ptr) = 0;

        SensorBase* getSensorPtr();

        //virtual void newKeyFrameCallback(FrameBase* _new_key_frame_ptr, const Scalar& _time_tolerance) = 0;

    private:
        ProcessorType type_;

};

inline bool ProcessorBase::permittedKeyFrame()
{
    return getWolfProblem()->permitKeyFrame(this);
}

inline SensorBase* ProcessorBase::getSensorPtr()
{
    return upperNodePtr();
}

} // namespace wolf

#endif
