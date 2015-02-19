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
    //variables
    std::list<Eigen::Vector2s> corners;
    
    //extract corners from range data
    extractCorners(corners);
    
    //generate a feature for each corner
    //TODO 
    
    //Establish correspondences for each feature
    //TODO (Does it go here ?)
    
}

unsigned int CaptureLaser2D::extractCorners(std::list<Eigen::Vector2s> & _corner_list)
{
    //local variables
    Eigen::VectorXs points(2,data_.size());
    Eigen::MatrixXs AA(3,3);
    double a00=0, a01=0, a02=0, a11=0, a12=0, azimuth, theta;
    Eigen::Vector3s line, corner, v001; 
    double error;
    std::list<Eigen::Vector3s> line_list;
    std::list<Eigen::Vector3s>::iterator line_it1, line_it2;
    std::list<unsigned int> index_list; 
    std::list<unsigned int>::iterator index_it1, index_it2;
    
    //cast to specialized sensor. TODO: Could be done once at the constructor ?
    SensorLaser2DPtr laser_ptr = (const SensorLaser2DPtr)sensor_ptr_;
    
    //init from sensor this->sensor_ptr_
    unsigned int n_rays = laser_ptr->getNumRays();
    double aperture = laser_ptr->getAperture();
    double azimuth_step = aperture/(double)n_rays;
    double range_std_dev = laser_ptr->getRangeStdDev();
    
    //other inits
    v001 << 0, 0, 1;
    
    //convert range polar data to cartesian points. Assumes clockwise order from the scan top view, and centered scan.
    for (unsigned int ii = 0; ii<data_.size(); ii++)
    {
        azimuth = -aperture/2.+(double)ii*azimuth_step;
        points.block<2,1>(0,ii) << data_(ii)*cos(azimuth), data_(ii)*sin(azimuth); //points.row0-> x coordinate, points.row1->y coordinate
    }
    
    //find line segments running over the scan
    for (unsigned int ii = segment_window_size-1; ii<data_.size(); ii++)
    {
        //Found the best fitting line over points within the window. Build the system: A*line=[0 0 1]'. Matrix A = a_ij
        for(unsigned int jj = 0; jj<segment_window_size; jj++) 
        {
            a00 += points(0,ii-jj)*points(0,ii-jj);//sum(x_i^2)
            a01 += points(0,ii-jj)*points(1,ii-jj);//sum(x_i*y_i)
            a02 += points(0,ii-jj);//sum(x_i)
            a11 += points(1,ii-jj)*points(1,ii-jj);//sum(y_i^2)
            a12 += points(1,ii-jj);//sum(y_i)
        }
        AA << a00, a01, a02, a01, a11, a12, 0, 0, 1;
        
        //solve for line
        line = AA.inverse()*v001; //assures normalized line result (last component set to 1)
        
        //compute error between line and supporting points
        error = 0;
        for(unsigned int jj = 0; jj<segment_window_size; jj++) 
        {
            error += fabs( line(0)*points(0,ii-jj) + line(1)*points(1,ii-jj) + line(2)) / sqrt( line(0)*line(0) + line(1)*line(1) ); 
        }
        
        //if error below stdev, add line to result set
        if ( error < range_std_dev*k_sigmas )
        {
            line_list.push_back(line); //keep the line in the result list
            index_list.push_back(ii); //keep the "last" point of the line in the index list
        }
    }
    
    //if at least two lines, find corners over the line list
    if ( line_list.size() > 1 )
    {
        line_it1 = line_list.begin();
        line_it2 = line_it1 ++;
        index_it1 = index_list.begin();
        index_it2 = index_it1 ++;
        while ( line_it1 != line_list.end() )
        {   
            //compute angle between lines 1 and 2
            theta = acos ( (*line_it1).dot(*line_it2) / (*line_it1).norm()*(*line_it2).norm() );
            
            //Check angle threshold and consecutiveness of lines in the scan
            if ( ( fabs(theta) > theta_min ) && ( ((*index_it2)-(*index_it1)) < segment_window_size ) )
            {
                corner = (*line_it1).cross(*line_it2); //cross product between lines is the intersection
                corner = corner / corner(2); //normalize point to set last component to 1
                _corner_list.push_back( Eigen::Vector2s(corner(0),corner(1)) );
            }
                    
            //update iterators
            line_it1 ++;
            line_it2 ++;
            index_it1 ++;
            index_it2 ++;
        }
    }
    
    return line_list.size();
}

Eigen::VectorXs CaptureLaser2D::computePrior() const
{
    return Eigen::Vector3s(1,2,3);
}

void CaptureLaser2D::findCorrespondences()
{
        //
}

