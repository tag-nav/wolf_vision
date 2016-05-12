/**
 * \file processor_tracker_landmark.h
 *
 *  Created on: Apr 7, 2016
 *      \author: jvallve
 */

#ifndef PROCESSOR_TRACKER_LANDMARK_H_
#define PROCESSOR_TRACKER_LANDMARK_H_

#include "processor_tracker.h"
#include "capture_base.h"
#include "wolf.h"

namespace wolf
{

/** \brief Landmark tracker processor
 *
 * This is an abstract class.
 *
 * This class implements the incremental landmark tracker.
 *
 * The incremental tracker contains three pointers to three Captures of type CaptureBase,
 * named \b origin, \b last and \b incoming:
 *   - \b origin: this points to a Capture where all Feature tracks start.
 *   - \b last: the last Capture tracked by the tracker.
 *     A sufficient subset of the Features in \b origin is still alive in \b last.
 *   - \b incoming: the capture being received. The tracker operates on this Capture,
 *     establishing correspondences between the features here and the features in \b origin.
 *     Each successful correspondence
 *     results in an extension of the track of the Feature up to the \b incoming Capture.
 *
 * This processor creates landmarks for new detected Features, and establishes constraints Feature-Landmark.
 *
 * This tracker builds on top of the ProcessorTracker by implementing some of its pure virtual functions.
 * As a reminder, we sketch here the pipeline of the parent ProcessorTracker process() function.
 * We highlight the functions implemented here with a sign  '<--- IMPLEMENTED', and the ones to be implemented by derived classes with '<=== IMPLEMENT'
 *
 *   - On each incoming Capture,
 *     - Track known features in the \b incoming Capture: processKnown()            <--- IMPLEMENTED
 *     - Check if enough Features are still tracked, and vote for a new KeyFrame if this number is too low:
 *     - if voteForKeyFrame()                                                       <=== IMPLEMENT
 *       - Populate the tracker with new Features : processNew()                    <--- IMPLEMENTED
 *       - Make a KeyFrame with the \b last Capture: makeFrame(), setKey()
 *       - Establish constraints of the new Features: establishConstraints()        <--- IMPLEMENTED
 *       - Reset the tracker with the \b last Capture as the new \b origin: reset() <--- IMPLEMENTED
 *     - else
 *       - Advance the tracker one Capture ahead: advance()                         <--- IMPLEMENTED
 *
 * The most important implemented methods are:
 *   - processKnown() : which calls the pure virtual, to be implemented in derived classes:
 *     - findLandmarks() : find Landmarks from the \b map in \b incoming            <=== IMPLEMENT
 *   - processNew() : which calls the pure virtuals:
 *     - detectNewFeatures() : detects new Features in \b last                      <=== IMPLEMENT
 *     - createLandmark() : creates a Landmark using a new Feature                  <=== IMPLEMENT
 *     - findLandmarks() : find the new Landmarks again in \b incoming              <=== IMPLEMENT
 *   - establishConstraints() : which calls the pure virtual:
 *     - createConstraint() : create a Feature-Landmark constraint of the correct derived type <=== IMPLEMENT
 *
 * Should you need extra functionality for your derived types, you can overload these two methods,
 *
 *   -  preProcess() { }
 *   -  postProcess() { }
 *
 * which are called at the beginning and at the end of process() respectively.
 * See the doc of these functions for more info.
 */
class ProcessorTrackerLandmark : public ProcessorTracker
{
    public:
        ProcessorTrackerLandmark(ProcessorType _tp, const unsigned int& _max_new_features = 0);
        virtual ~ProcessorTrackerLandmark();

    protected:

        LandmarkMatchMap matches_landmark_from_incoming_;
        LandmarkMatchMap matches_landmark_from_last_;

        /** \brief Tracker function
         * \return The number of successful tracks.
         *
         * This is the tracker function to be implemented in derived classes.
         * It operates on the \b incoming capture pointed by incoming_ptr_.
         *
         * This should do one of the following, depending on the design of the tracker:
         *   - Track Features against other Features in the \b origin Capture. Tips:
         *     - An intermediary step of matching against Features in the \b last Capture makes tracking easier.
         *     - Once tracked against last, then the link to Features in \b origin is provided by the Features' Constraints in \b last.
         *     - If required, correct the drift by re-comparing against the Features in origin.
         *     - The Constraints in \b last need to be transferred to \b incoming (moved, not copied).
         *
         * The function must generate the necessary Features in the \b incoming Capture,
         * of the correct type, derived from FeatureBase.
         *
         * It must also generate the constraints, of the correct type, derived from ConstraintBase
         * (through ConstraintAnalytic or ConstraintSparse).
         */
        virtual unsigned int processKnown();

        /** \brief Find provided landmarks in the incoming capture
         * \param _landmark_list_in input list of landmarks to be found in incoming
         * \param _feature_list_out returned list of incoming features corresponding to a landmark of _landmark_list_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         */
        virtual unsigned int findLandmarks(const LandmarkBaseList& _landmark_list_in, FeatureBaseList& _feature_list_out,
                                           LandmarkMatchMap& _feature_landmark_correspondences) = 0;

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame() = 0;

        // We overload the advance and reset functions to update the lists of matches
        void advance();
        void reset();

        /**\brief Process new Features
         *
         */
        unsigned int processNew(const unsigned int& _max_features);

        /** \brief Detect new Features
         * \param _capture_ptr Capture for feature detection. Defaults to incoming_ptr_.
         * \param _new_features_list The list of detected Features. Defaults to member new_features_list_.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function sets the member new_features_list_, the list of newly detected features,
         * to be used for landmark initialization.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_features) = 0;

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBase* createLandmark(FeatureBase* _feature_ptr) = 0;

        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         *
         * Implement this method in derived classes.
         *
         * TODO: Make a general ConstraintFactory, and put it in WolfProblem.
         * This factory only needs to know the two derived pointers to decide on the actual Constraint created.
         */
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr) = 0;

        /** \brief Establish constraints between features in Captures \b last and \b origin
         */
        virtual void establishConstraints();

};

}// namespace wolf

// IMPLEMENTATION

#include <utility>
namespace wolf
{
inline void ProcessorTrackerLandmark::advance()
{
    std::cout << "ProcessorTrackerLandmark::advance" << std::endl;
    matches_landmark_from_last_ = std::move(matches_landmark_from_incoming_);

    new_features_last_ = std::move(new_features_incoming_);

//    for (auto match : matches_landmark_from_last_)
//            std::cout << "\t" << match.first->id() << " to " << match.second.landmark_ptr_->id() << std::endl;
}

inline void ProcessorTrackerLandmark::reset()
{
    std::cout << "ProcessorTrackerLandmark::reset" << std::endl;
    matches_landmark_from_last_ = std::move(matches_landmark_from_incoming_);

    new_features_last_ = std::move(new_features_incoming_);

//    for (auto match : matches_landmark_from_last_)
//            std::cout << "\t" << match.first->id() << " to " << match.second.landmark_ptr_->id() << std::endl;
}

inline void ProcessorTrackerLandmark::establishConstraints()
{
    std::cout << "ProcessorTrackerLandmark::establishConstraints" << std::endl;
    std::cout << "\tfeatures:" << last_ptr_->getFeatureListPtr()->size() << std::endl;
    std::cout << "\tcorrespondences: " << matches_landmark_from_last_.size() << std::endl;

    for (auto last_feature : *(last_ptr_->getFeatureListPtr()))
        last_feature->addConstraint(createConstraint(last_feature, matches_landmark_from_last_[last_feature].landmark_ptr_));
}

}// namespace wolf

#endif /* PROCESSOR_TRACKER_LANDMARK_H_ */
