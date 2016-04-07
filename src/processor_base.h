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

// Std includes


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

        SensorBase* getSensorPtr();

        virtual void init(CaptureBase* _capture_ptr) = 0;

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

    private:
        ProcessorType type_;

};

inline SensorBase* ProcessorBase::getSensorPtr()
{
    return upperNodePtr();
}

inline bool ProcessorBase::permittedKeyFrame()
{
    return getWolfProblem()->permitKeyFrame(this);
}

} // namespace wolf

#endif
