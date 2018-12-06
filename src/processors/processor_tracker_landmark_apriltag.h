#ifndef _PROCESSOR_TRACKER_LANDMARK_APRILTAG_H_
#define _PROCESSOR_TRACKER_LANDMARK_APRILTAG_H_

//Wolf includes
#include "wolf.h"
#include "processor_tracker_landmark.h"

// Apriltag
#include <apriltag.h>

// open cv
#include <opencv/cv.h>


namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsApriltag);
struct ProcessorParamsApriltag : public ProcessorParamsTrackerLandmark
{
    //tag parameters
    std::string tag_family_;
    int tag_black_border_;

    //detector parameters
    Scalar quad_sigma_;
    unsigned int nthreads_;
    bool debug_;
    bool refine_edges_;
    bool refine_decode_;
    bool refine_pose_;
};



WOLF_PTR_TYPEDEFS(ProcessorTrackerLandmarkApriltag);

class ProcessorTrackerLandmarkApriltag : public ProcessorTrackerLandmark
{
    public:


        /** \brief Class constructor
         */
        // TODO Modify this default API to suit your class needs
        ProcessorTrackerLandmarkApriltag( ProcessorParamsApriltagPtr _params_tracker_landmark_apriltag);

        /** \brief Class Destructor
         */
        virtual ~ProcessorTrackerLandmarkApriltag();

        void preProcess();

        /** \brief Find provided landmarks in the incoming capture
         * \param _landmark_list_in input list of landmarks to be found in incoming
         * \param _feature_list_out returned list of incoming features corresponding to a landmark of _landmark_list_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         */
        virtual unsigned int findLandmarks(const LandmarkBaseList& _landmark_list_in, FeatureBaseList& _feature_list_out,
                                           LandmarkMatchMap& _feature_landmark_correspondences);

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame();

        /** \brief Detect new Features
         * \param _max_features The maximum number of features to detect.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function sets the member new_features_list_, the list of newly detected features,
         * to be used for landmark initialization.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_features, FeatureBaseList& _feature_list_out);

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBasePtr createLandmark(FeatureBasePtr _feature_ptr);

        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         *
         * Implement this method in derived classes.
         */
        virtual ConstraintBasePtr createConstraint(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr);

    private:
        cv::Mat grayscale_image_;
        apriltag_detector_t detector_;
        FeatureBaseList detections_incoming_;
        FeatureBaseList detections_last_;
};

} // namespace wolf

#endif /* _PROCESSOR_TRACKER_LANDMARK_APRILTAG_H_ */
