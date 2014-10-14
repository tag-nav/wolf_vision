#include "gaussian.h"

Gaussian::Gaussian(const unsigned int _size) :
        DensityBase(_size) //
{
}

Gaussian::~Gaussian()
{
}

MatrixXs Gaussian::covariance()
{
    return covariance_;
}
