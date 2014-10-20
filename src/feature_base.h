/**
 * \file feature_base.h
 *
 *  Created on: 12/08/2014
 *     \author: jsola
 */

#ifndef FEATURE_BASE_H_
#define FEATURE_BASE_H_

// Forward declarations for node templates
class CaptureBase;
class CorrespondenceBase;

// Forward declarations for member pointers
// class SensorBase;
// class RawBase;
// class TransSensor;

//wolf includes
#include "node_constrainer.h"

class FeatureBase : public NodeConstrainer<Capture, CorrespondenceBase>
{

    protected:
        Eigen::VectorXs measurement_;

    protected:

        /** \brief Constructor
         * 
         * \param _capt_ptr a shared pointer to the Capture up node
         * \param _dim_measurement the dimension of the measurement space
         * \param _loc the location in the Wolf tree (TOP, MID or BOTTOM, it defaults to MID)
         * 
         */
        FeatureBase(const CaptureShPtr& _capt_ptr, unsigned int _dim_measurement, const NodeLocation _loc = MID);

        virtual ~FeatureBase();

    public:

        void linkToCapture(const CaptureShPtr& _capt_ptr);

        const CapturePtr capturePtr() const;
        const Capture& capture() const;

        void addCorrespondence(const CorrespondenceShPtr& _co_ptr);

        const CorrespondenceList& correspondenceList() const;

        const Eigen::VectorXs& measurement() const;
        
        /** \brief Generic interface to find correspondences
         * 
         * TBD
         * Generic interface to check correspondence between this feature and a map model (static/slam) or a previous feature
         *
         **/
        virtual void checkCorrespondence();
        
        /** \brief prints object's info
         * 
         * prints object's info
         * 
         **/
        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;

};

//////////////////////////////////////
// IMPLEMENTATION
//////////////////////////////////////

FeatureBase::FeatureBase(const CaptureShPtr& _capt_ptr, unsigned int _dim_measurement, const NodeLocation _loc) :
        NodeConstrainer(_loc, "FEATURE", _capt_ptr), //
        measurement_(_dim_measurement)
{
}

FeatureBase::~FeatureBase()
{
    //
}

inline void FeatureBase::linkToCapture(const CaptureShPtr& _capt_ptr)
{
    linkToUpNode(_capt_ptr);
}

inline const CapturePtr FeatureBase::capturePtr() const
{
    return upNodePtr();
}

inline const Capture& FeatureBase::capture() const
{
    return *capturePtr();
}

inline void FeatureBase::addCorrespondence(const CorrespondenceShPtr& _co_ptr)
{
    addDownNode(_co_ptr);
}

inline const CorrespondenceList& FeatureBase::correspondenceList() const
{
    return downNodeList();
}

inline const Eigen::VectorXs& FeatureBase::measurement() const
{
    return measurement_;
}

inline void FeatureBase::checkCorrespondence()
{
    //nothing to do in base class. Overload it to do things.
}

void FeatureBase::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    NodeConstrainer::printSelf(_ntabs, _ost);
    printNTabs(_ntabs);
    _ost << "\tMeasurement: ( " << measurement().transpose() << " )" << std::endl;
}

#endif /* FEATURE_BASE_H_ */
