
#include "landmark_line_2D.h"

namespace wolf {

LandmarkLine2D::LandmarkLine2D(StateBlock* _p_ptr, Eigen::Vector2s & _point1, Eigen::Vector2s & _point2) :
    LandmarkBase(LANDMARK_LINE_2D, _p_ptr), 
    
{
    setType("LINE_2D");
    
    //set descriptor as the Euclidean coordinates of extreme points
    descriptor_.resize(4);
    descriptor_.head(2) = _point1; 
    descriptor_.tail(2) = _point2; 
    
    //set the eigen mappings to point 1 and 2
    point1_ = 
}

LandmarkLine2D::~LandmarkLine2D()
{
    //
}

void LandmarkLine2D::updateExtremePoints(Eigen::VectorXs & _point1, Eigen::VectorXs & _point2)
{
    //TODO
}

const Eigen::Vector2s & LandmarkLine2D::point1() const
{
    return descriptor_.head(2); 
}

const Eigen::Vector2s & LandmarkLine2D::point2() const
{
    return descriptor_.tail(2); 
}

} // namespace wolf
