#ifndef PROCESSOR_BASE_H_
#define PROCESSOR_BASE_H_

// Fwd refs
class SensorBase;
class NodeTerminus;

//Wolf includes
#include "wolf.h"
#include "node_linked.h"

// Std includes

//class ProcessorBase
class ProcessorBase : public NodeLinked<SensorBase, NodeTerminus>
{
    public:
        /** \brief Constructor
         *
         * Constructor
         *
         **/
        ProcessorBase();

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ProcessorBase();

        SensorBase* getSensorPtr();

        /** \brief Extract Features
         *
         * Extract Features from a given capture
         *
         **/
        virtual void extractFeatures(CaptureBase* _capture_ptr) = 0;

        /** \brief Establish Constraints
         *
         * Establish Constraints for all features of a given capture
         *
         **/
        virtual void establishConstraints(CaptureBase* _capture_ptr) = 0;

};
#endif
