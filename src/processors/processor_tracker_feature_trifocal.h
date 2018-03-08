#ifndef _PROCESSOR_TRACKER_FEATURE_TRIFOCAL_H_
#define _PROCESSOR_TRACKER_FEATURE_TRIFOCAL_H_

//Wolf includes
#include "wolf.h"
#include "processor_tracker_feature.h"

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsTrackerFeatureTrifocal);

struct ProcessorParamsTrackerFeatureTrifocal : public ProcessorParamsTracker
{
        std::string yaml_file_params_vision_utils;

        unsigned int min_features_for_keyframe; ///< minimum nbr. of features to vote for keyframe

        Scalar pixel_noise_std; ///< std noise of the pixel
        Scalar pixel_noise_var; ///< var noise of the pixel
};

WOLF_PTR_TYPEDEFS(ProcessorTrackerFeatureTrifocal);

class ProcessorTrackerFeatureTrifocal : public ProcessorTrackerFeature
{

    public: // TODO make private
        CaptureBasePtr prev_origin_ptr_;                    ///< Capture previous to origin_ptr_ for the third focus of the trifocal.
        bool initialized_;                                  ///< flags the situation where three focus are available: prev_origin, origin, and last.
        FeatureMatchMap matches_prev_origin_from_origin_;   ///< matches between prev_origin and origin.

    public:

        /** \brief Class constructor
         */
        ProcessorTrackerFeatureTrifocal( const ProcessorParamsTrackerFeatureTrifocal& _params );

        /** \brief Class Destructor
         */
        virtual ~ProcessorTrackerFeatureTrifocal();

        /** \brief Track provided features from \b last to \b incoming
         * \param _feature_list_in input list of features in \b last to track
         * \param _feature_list_out returned list of features found in \b incoming
         * \param _feature_correspondences returned map of correspondences: _feature_correspondences[feature_out_ptr] = feature_in_ptr
         */
        virtual unsigned int trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out, FeatureMatchMap& _feature_correspondences);

        /** \brief Correct the drift in incoming feature by re-comparing against the corresponding feature in origin.
         * \param _origin_feature input feature in origin capture tracked
         * \param _incoming_feature input/output feature in incoming capture to be corrected
         * \return false if the the process discards the correspondence with origin's feature
         */
        virtual bool correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature);

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame();

        /** \brief Detect new Features
         * \param _max_features maximum number of features detected
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function sets the member new_features_last_, the list of newly detected features.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_features);

        /** \brief Create a new constraint and link it to the wolf tree
         * \param _feature_ptr pointer to the parent Feature
         * \param _feature_other_ptr pointer to the other feature constrained.
         *
         * Implement this method in derived classes.
         *
         * This function creates a constraint of the appropriate type for the derived processor.
         */
        virtual ConstraintBasePtr createConstraint(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr);


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief reset pointers and match lists at KF creation
         *
         */
        virtual void resetDerived();

        /** \brief Pre-process: check if all captures (prev-origin, origin, last) are initialized to allow constraints creation
         *
         */
        virtual void preProcess();

        /** \brief Establish constraints between features in Captures \b last and \b origin
         */
        virtual void establishConstraints();

};

} // namespace wolf

#endif /* _PROCESSOR_TRACKER_FEATURE_TRIFOCAL_H_ */
