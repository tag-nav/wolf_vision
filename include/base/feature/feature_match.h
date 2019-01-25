#ifndef FEATURE_MATCH_H_
#define FEATURE_MATCH_H_

// Wolf includes
#include "wolf.h"

//wolf nampseace
namespace wolf {
    
//forward declaration to typedef class pointers
WOLF_STRUCT_PTR_TYPEDEFS(FeatureMatch);

/** \brief Map of feature matches
 *
 *   corr feature  <--------------  actual feature
 *
 * map<FeatureBasePtr actual_feature, FeatureMatchPtr corresponding_feature_match>
 *
 */
typedef std::map<FeatureBasePtr, FeatureMatchPtr> FeatureMatchMap; //a map is also typedefined  
    
/** \brief Match between a feature and a feature
 *
 * Match between a feature and a feature (feature-feature correspondence)
 *
 */
struct FeatureMatch
{
        FeatureBasePtr feature_ptr_; ///< Corresponding feature
        Scalar normalized_score_;    ///< normalized similarity score (0 is bad, 1 is good)
};

}//end namespace

#endif


