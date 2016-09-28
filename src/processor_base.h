#ifndef PROCESSOR_BASE_H_
#define PROCESSOR_BASE_H_

// Fwd refs
namespace wolf{
class SensorBase;
class NodeTerminus;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"
#include "problem.h"

namespace wolf {



/** \brief base struct for processor parameters
 *
 * Derive from this struct to create structs of processor parameters.
 */
struct ProcessorParamsBase
{
        std::string type;
        std::string name;
};

//class ProcessorBase
class ProcessorBase : public NodeBase // NodeLinked<SensorBase, NodeTerminus>
{
    private:
        Problem* problem_ptr_;
        SensorBase* sensor_ptr_;
    public:
        ProcessorBase(ProcessorType _tp, const std::string& _type, const Scalar& _time_tolerance = 0);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ProcessorBase();

        unsigned int id();

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

        /**\brief make a Frame with the provided Capture
         */
        virtual void makeFrame(CaptureBase* _capture_ptr, FrameKeyType _type = NON_KEY_FRAME);

        virtual bool keyFrameCallback(FrameBase* _keyframe_ptr, const Scalar& _time_tolerance) = 0;

        SensorBase* getSensorPtr();
        const SensorBase* getSensorPtr() const;
        void setSensorPtr(SensorBase* _sen_ptr){sensor_ptr_ = _sen_ptr;}

        virtual bool isMotion();

        Problem* getProblem(){return problem_ptr_;}
        void setProblem(Problem* _prob_ptr){problem_ptr_ = _prob_ptr;}

    private:
        static unsigned int processor_id_count_;

    protected:
        unsigned int processor_id_;
        ProcessorType type_id_;
        Scalar time_tolerance_;         ///< self time tolerance for adding a capture into a frame
};

}

#include "constraint_base.h"

namespace wolf {

inline bool ProcessorBase::isMotion()
{
    return false;
}

}

//#include "problem.h"

namespace wolf {

inline unsigned int ProcessorBase::id()
{
    return processor_id_;
}

inline SensorBase* ProcessorBase::getSensorPtr()
{
    return sensor_ptr_;
//    return upperNodePtr();
}

inline const SensorBase* ProcessorBase::getSensorPtr() const
{
    return sensor_ptr_;
//    return upperNodePtr();
}

} // namespace wolf

#endif
