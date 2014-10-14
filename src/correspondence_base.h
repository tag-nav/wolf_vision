/**
 * \file correspondence_base.h
 *
 *  Created on: 12/08/2014
 *     \author: jsola
 */

#ifndef CORRESPONDENCE_BASE_H_
#define CORRESPONDENCE_BASE_H_

#include "node_constrainer.h"

// Forward declarations for node templates
class FeatureBase;
class NodeTerminus;

// fwd dec for member pointers
class SensorBase;

class CorrespondenceBase : public NodeConstrainer<FeatureBase, NodeTerminus>
{

    protected:
        // own data stored here
        Eigen::VectorXs expectation_; ///< the expected measurement given the current state estimate

        // NOTE: We access "own" data stored somewhere else up in the wolf tree
        // through accessor functions by traversing the tree up: starting at featurePtr()-> and navigating to the requested data.
        // See e.g. sensorPtr() or sensor().

        // NOTE: Access "other" data through pointers and/or accessor functions in derived classes,
        // depending on the type of corresponded object.
        // See e.g. CorrespondenceRelative.

    protected:

        /**
         * \brief Constructor
         * \param _ft_ptr a shared pointer to the FeatureBase up node
         * \param _dim_error the dimension of the error vector
         * \param _dim_expectation the dimension of the expectation vector
         */
        CorrespondenceBase(const FeatureShPtr& _ft_ptr, unsigned int _dim_error, unsigned int _dim_expectation);

        virtual ~CorrespondenceBase();

    public:

        void linkToFeature(const FeatureShPtr& _ft_ptr);

        const FeaturePtr featurePtr() const;
        const FeatureBase& feature() const;

        const CapturePtr capturePtr() const;
        const Capture& capture() const;

        const SensorPtr sensorPtr() const;
        const SensorBase& sensor() const;

        const StatePtr statePtr() const;
        const StatePose& state() const;

        const Eigen::VectorXs& expectation() const;

        /**
         * \brief Compute measurement expectation.
         *
         * Overload this function in derived classes to implement the measurement model h(X).
         *
         * The measurement model h(X) produces an expected measurement given the current state estimate
         * and other possible data such as previous measurements of the same features:
         *   - expectation = computeExpectation() = h (state_, other_)
         *
         * The input parameters state_ and other_ are known by class Correspondence and its derivates.
         *
         */
        virtual void computeExpectation() = 0;

        /**
         * \brief Compute error of this correspondence (non recursive)
         *
         * Compute error of this correspondence (non recursive).
         *
         * This function is implemented here in its default form, i.e., it first computes the expectation
         * for the correspondence, and then
         * the error is the difference between the measurement and the expectation.
         *
         * This method can be overloaded in derived classes to allow for different error computation methods.
         *
         * However, if you only need to specify a measurement model, and hence you are satisfied
         * with the default error computation,
         * it is better to overload only the computeExpectation() function.
         */
        virtual void computeError();

        /**
         * \brief Compute error of this correspondence (non recursive)
         *
         * Compute the error for the node and place it at the given \a _residuals vector,
         * starting at position \a _idx and spanning the dimension of the error of this node.
         *
         * \sa computeError() for more information.
         */
        virtual void computeError(Eigen::VectorXs & _residuals, unsigned int & _idx);

        /**
         * \brief Compute error of this correspondence (non recursive)
         *
         * Compute the error for the node and place it at the given \a _residuals Map<VectorXs>,
         * starting at position \a _idx and spanning the dimension of the error of this node.
         *
         * \sa computeError() for more information.
         */
        virtual void computeError(Eigen::Map<Eigen::VectorXs> & _residuals, unsigned int & _idx);

        /**
         * \brief Compute error of this correspondence (non recursive)
         *
         * Compute the error for the node and place it at the given \a _residuals pointer
         * plus \a _idx and spanning the dimension of the error of this node.
         *
         * \sa computeError() for more information.
         */
        virtual void computeError(WolfScalar * _residuals, unsigned int & _idx);

        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;

};


//////////////////////////////////////
// IMPLEMENTATION
//////////////////////////////////////

#include "sensor_base.h"


CorrespondenceBase::CorrespondenceBase(const FeatureShPtr& _ft_ptr, unsigned int _dim_error,
                                              unsigned int _dim_expectation) :
        NodeConstrainer(BOTTOM, "CORRESPONDENCE", _ft_ptr, _dim_error), //
        expectation_(_dim_expectation) //
{
    //
}

CorrespondenceBase::~CorrespondenceBase()
{
    //
}

void CorrespondenceBase::linkToFeature(const FeatureShPtr& _ft_ptr)
{
    linkToUpNode(_ft_ptr);
}

inline const FeaturePtr CorrespondenceBase::featurePtr() const
{
    return upNodePtr();
}

inline const FeatureBase& CorrespondenceBase::feature() const
{
    return *featurePtr();
}

inline const CapturePtr CorrespondenceBase::capturePtr() const
{
    return featurePtr()->capturePtr();
}

inline const Capture& CorrespondenceBase::capture() const
{
    return featurePtr()->capture();
}

inline const SensorPtr CorrespondenceBase::sensorPtr() const
{
    return featurePtr()->capturePtr()->sensorPtr();
}

inline const SensorBase& CorrespondenceBase::sensor() const
{
    return featurePtr()->capturePtr()->sensor();
}

inline const StatePtr CorrespondenceBase::statePtr() const
{
    return featurePtr()->capturePtr()->framePtr()->statePtr();
}

inline const StatePose& CorrespondenceBase::state() const
{
    return featurePtr()->capturePtr()->framePtr()->state();
}

inline const Eigen::VectorXs& CorrespondenceBase::expectation() const
{
    return expectation_;
}


void CorrespondenceBase::computeError()
{
    computeExpectation();
    assert(featurePtr()->measurement().size() == expectation().size());
    error_ = featurePtr()->measurement() - expectation();
}

inline void CorrespondenceBase::computeError(Eigen::VectorXs & _residuals, unsigned int & _idx)
{
    computeExpectation();
    assert(featurePtr()->measurement().size() == expectation().size());
    _residuals.segment(_idx, dim_error_) = featurePtr()->measurement() - expectation();
    _idx += dim_error_;
}

inline void CorrespondenceBase::computeError(Eigen::Map<Eigen::VectorXs> & _residuals, unsigned int & _idx)
{
    computeExpectation();
    assert(featurePtr()->measurement().size() == expectation().size());
    _residuals.segment(_idx, dim_error_) = featurePtr()->measurement() - expectation();
    _idx += dim_error_;
}

inline void CorrespondenceBase::computeError(WolfScalar * _residuals, unsigned int & _idx)
{
    computeExpectation();
    assert(featurePtr()->measurement().size() == expectation().size());
    Eigen::Map<Eigen::VectorXs>(_residuals + _idx, dim_error_) = featurePtr()->measurement() - expectation();
    _idx += dim_error_;
}

void CorrespondenceBase::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    NodeConstrainer<FeatureBase,NodeTerminus>::printSelf(_ntabs, _ost);
    printNTabs(_ntabs);
    _ost << "\tExpectation: ( " << expectation_.transpose() << " )" << std::endl;
}

#endif /* CORRESPONDENCE_BASE_H_ */
