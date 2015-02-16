#include "capture_laser_2D.h"

unsigned int CaptureLaser2D::segment_window_size = 8;//window size to extract segments
double CaptureLaser2D::theta_min = 0.52; //minimum theta between consecutive segments to detect corner. PI/6=0.52
double CaptureLaser2D::k_sigmas = 3.;//How many std_dev are tolerated to count that a point is supporting a line

CaptureLaser2D::CaptureLaser2D(const TimeStamp & _ts, const SensorLaser2DPtr & _sensor_ptr, const Eigen::VectorXs& _ranges):
    CaptureBase(_ts, _sensor_ptr, _ranges)
{
    // 
}

CaptureLaser2D::~CaptureLaser2D()
{
    //
}

void CaptureLaser2D::processCapture()
{
    extractCorners();
}

void CaptureLaser2D::extractCorners()
{
    //local vars
    Eigen::VectorXs points(2,data_.size());
    Eigen::MatrixXs AA(3,3);
    double a00=0, a01=0, a02=0, a11=0, a12=0, azimuth;
    
    //cast to specialized sensor. TODO: Could be done once at the constructor ?
    SensorLaser2DPtr laser_ptr = (const SensorLaser2DPtr)sensor_ptr_;
    
    //init from sensor this->sensor_ptr_
    unsigned int n_rays = laser_ptr->getNumRays();
    double aperture = laser_ptr->getAperture();
    double azimuth_step = aperture/(double)n_rays;
    
    //convert data to cartesian points. Assuming clockwise order from the scan top view
    for (unsigned int ii = 0; ii<data_.size(); ii++)
    {
        azimuth = -aperture/2.+(double)ii*azimuth_step;
        points.block(0,ii,2,1) << data_(ii)*cos(azimuth), data_(ii)*sin(azimuth); //row0-> x coordinate, row1->y coordinate
    }
    
    //find corners running over the scan
    for (unsigned int ii = segment_window_size-1; ii<data_.size(); ii++)
    {
        //Found the best fitting line over the window. Build the system: Ax=0. Matrix A = a_ij
        for(unsigned int jj = 0; jj<segment_window_size; jj++) 
        {
            a00 += points(0,ii-jj)*points(0,ii-jj);//sum(x_i^2)
            a01 += points(0,ii-jj)*points(1,ii-jj);//sum(x_i*y_i)
            a02 += points(0,ii-jj);//sum(x_i)
            a11 += points(1,ii-jj)*points(1,ii-jj);//sum(y_i^2)
            a12 += points(1,ii-jj);//sum(y_i)
        }
        AA << a00, a01, a02, a01, a11, a12, 0, 0, 1;
        
        //solve
        
    }
}
