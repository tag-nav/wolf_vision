#include "ceres_wrapper_numeric.h"

CeresManager::CeresManager()
{
}
 
CeresManager::~CeresManager()
{
}

// template <typename T> bool operator()(const T* const x, T* residual) const 
// {
//     residual[0] = T(10.0) - x[0];
//     return true;
// }
// 
