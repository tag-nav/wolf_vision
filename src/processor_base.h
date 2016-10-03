#ifndef PROCESSOR_BASE_H_
#define PROCESSOR_BASE_H_

// Fwd refs
namespace wolf{
class SensorBase;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"

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
        ProblemPtr problem_ptr_;
        SensorBasePtr sensor_ptr_;
    public:
        ProcessorBase(ProcessorType _tp, const std::string& _type, const Scalar& _time_tolerance = 0);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ProcessorBase();
        void destruct();

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
         */
        virtual void makeFrame(CaptureBasePtr _capture_ptr, FrameKeyType _type = NON_KEY_FRAME);

        virtual bool keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tolerance) = 0;

        SensorBasePtr getSensorPtr();
        const SensorBasePtr getSensorPtr() const;
        void setSensorPtr(SensorBasePtr _sen_ptr){sensor_ptr_ = _sen_ptr;}

        virtual bool isMotion();

        ProblemPtr getProblem();
        void setProblem(ProblemPtr _prob_ptr){problem_ptr_ = _prob_ptr;}

    private:
        static unsigned int processor_id_count_;

    protected:
        unsigned int processor_id_;
        ProcessorType type_id_;
        Scalar time_tolerance_;         ///< self time tolerance for adding a capture into a frame
};

}

#include "sensor_base.h"
#include "constraint_base.h"

namespace wolf {

inline wolf::ProblemPtr ProcessorBase::getProblem()
{
    if (problem_ptr_ == nullptr && sensor_ptr_ != nullptr)
        problem_ptr_ = sensor_ptr_->getProblem();
    return problem_ptr_;

}

inline void ProcessorBase::destruct()
{
    if (!is_deleting_)
    {
        if (sensor_ptr_ != nullptr) // && !up_node_ptr_->isTop())
        {
            //std::cout << "upper node is not WolfProblem " << std::endl;
            sensor_ptr_->removeProcessor(this);
        }
        else
        {
            //std::cout << "upper node is WolfProblem or nullptr" << std::endl;
            delete this;
        }
    }
}


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
    return sensor_ptr_;
//    return upperNodePtr();
}

inline const SensorBasePtr ProcessorBase::getSensorPtr() const
{
    return sensor_ptr_;
//    return upperNodePtr();
}

} // namespace wolf

#endif
