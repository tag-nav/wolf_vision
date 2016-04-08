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



// Correspondence Landmark - Feature
typedef std::map<LandmarkBase*, FeatureBase*> FeatureLandmarkMap;

class ProcessorTrackerLandmark : public ProcessorTracker
{
    public:
        ProcessorTrackerLandmark(ProcessorType _tp);
        virtual ~ProcessorTrackerLandmark();

    protected:

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
        virtual unsigned int processKnown()
        {
            // Track features from last_ptr_ to incoming_ptr_
            FeatureLandmarkMap incoming_2_landmark;
            FeatureBaseList feature_incoming_found;
            unsigned int tracked_features = findLandmarks(*(last_ptr_->getFeatureListPtr()), feature_incoming_found, incoming_2_landmark);

            // Check/correct incoming-origin correspondences
            for (auto incoming_feature : feature_incoming_found)
            {
                assert(incoming_2_landmark[incoming_feature]->getConstraintListPtr()->size() == 1
                        && "More than 1 constraint in a feature after tracking!");

                // Check and correct the correspondence
                if (correctFeatureCorrespondence(incoming_feature, incoming_2_landmark[incoming_feature]))
                {
                    // Correspondence corrected -> add traked feature to infoming capture & move constraint
                    // 1. Add tracked feature to incoming capture
                    incoming_ptr_->addFeature(incoming_feature);

                    ConstraintBase* constraint_last_to_origin_ = incoming_2_landmark[incoming_feature]->getConstraintListPtr()->front();
                    // 2. Unlink constraint from last feature
                    incoming_2_landmark[incoming_feature]->unlinkDownNode(incoming_2_landmark[incoming_feature]->getConstraintListPtr()->begin());
                    // 3. Add constraint to incoming feature
                    incoming_feature->addConstraint(constraint_last_to_origin_);
                }
                else
                {
                    // Correspondence not confirmed -> Remove incoming feature & ignore correspondence
                    incoming_feature->destruct();
                }
            }
            return tracked_features;
        }



        /** \brief Find provided features in the map
         * \param _landmark_list_in input list of landmarks to be found in incoming
         * \param _feature_list_out returned list of incoming features corresponding to a landmark of _landmark_list_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         */
        virtual unsigned int findLandmarks(LandmarkBaseList& _landmark_list_in, FeatureBaseList& _feature_list_out, FeatureLandmarkMap _feature_landmark_correspondences) = 0;

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

        unsigned int processNew()
        {
            /* Rationale: A keyFrame will be created using the last Capture.
             * First, we work on this Capture to detect new Features,
             * eventually create Landmarks with them,
             * and in such case create the new Constraints feature-landmark.
             * When done, we need to track these new Features to the incoming Capture.
             * At the end, all new Features are appended to the lists of known Features in
             * the last and incoming Captures.
             */
            // We first need to populate the \b last Capture with new Features
            unsigned int n = detectNewFeatures();
            // TODO: class member?
            LandmarkBaseList new_landmarks;
            for (FeatureBase* feature_ptr : new_features_list_last_)
            {
                // create new landmark
                LandmarkBase* new_lmk_ptr = createLandmark(feature_ptr);
                new_landmarks.push_back(new_lmk_ptr);
                // create and add constraint
                ConstraintBase* constr_ptr = createConstraint(feature_ptr, new_lmk_ptr);
                feature_ptr->addConstraint(constr_ptr);
            }

            // Track new features from last to incomming (that will become origin and last)
            LandmarkFeatureMap landmark_2_feature;
            findLandmarks(new_features_list_incoming_, new_landmarks, landmark_2_feature);

            // Append all new Features to the Capture's list of Features
            last_ptr_->getFeatureListPtr()->splice(last_ptr_->getFeatureListPtr()->end(), new_features_list_last_);
            incoming_ptr_->getFeatureListPtr()->splice(incoming_ptr_->getFeatureListPtr()->end(), new_features_list_incoming_);

            // TODO: append new landmarks
            //getWolfProblem()->addLandmark();

            // return the number of new features detected in \b last
            return n;
        }
};

#endif /* PROCESSOR_TRACKER_LANDMARK_H_ */
