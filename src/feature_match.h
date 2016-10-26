#ifndef FEATURE_MATCH_H_
#define FEATURE_MATCH_H_

// Wolf includes
#include "wolf.h"

//wolf nampseace
namespace wolf {
    
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

typedef std::shared_ptr<FeatureMatch> FeatureMatchPtr;
typedef std::map<FeatureBasePtr, FeatureMatchPtr> FeatureMatchMap;

}//end namespace

#endif


