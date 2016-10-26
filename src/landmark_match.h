#ifndef LANDMARK_MATCH_H_
#define LANDMARK_MATCH_H_

// Wolf includes
#include "wolf.h"

//wolf nampseace
namespace wolf {
    
/** \brief Match between a feature and a landmark
 *
 * Match between a feature and a landmark
 *
 **/
struct LandmarkMatch
{
    LandmarkBasePtr landmark_ptr_;
    Scalar normalized_score_;
    
    LandmarkMatch() :
            landmark_ptr_(nullptr), normalized_score_(0)
    {

    }
    LandmarkMatch(LandmarkBasePtr _landmark_ptr, Scalar _normalized_score) :
            landmark_ptr_(_landmark_ptr), normalized_score_(_normalized_score)
    {

    }
};

// Map of Feature - Landmark matches
typedef std::shared_ptr<LandmarkMatch> LandmarkMatchPtr;
typedef std::map<FeatureBasePtr, LandmarkMatchPtr> LandmarkMatchMap;

}//end namespace

#endif
