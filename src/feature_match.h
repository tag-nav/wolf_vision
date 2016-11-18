#ifndef FEATURE_MATCH_H_
#define FEATURE_MATCH_H_

// Wolf includes
#include "wolf.h"

//wolf nampseace
namespace wolf {
    
//forward declaration to typedef class pointers
struct FeatureMatch;
typedef std::shared_ptr<FeatureMatch> FeatureMatchPtr;
typedef std::shared_ptr<const FeatureMatch> FeatureMatchConstPtr;
typedef std::weak_ptr<FeatureMatch> FeatureMatchWPtr;
typedef std::map<FeatureBasePtr, FeatureMatchPtr> FeatureMatchMap; //a map is also typedefined  
    
/** \brief Match between a feature and a feature
 *
 * Match between a feature and a feature (feature-feature correspondence)
 *
 **/
struct FeatureMatch
{
        FeatureBasePtr feature_ptr_;
        Scalar normalized_score_;
};

}//end namespace

#endif


