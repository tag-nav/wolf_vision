
#ifndef CAPTURE_LASER_2D_H_
#define CAPTURE_LASER_2D_H_

//wolf forward declarations
namespace wolf{
class SensorLaser2D;
}

//wolf includes
#include "capture_base.h"
#include "sensor_laser_2D.h"

//laserscanutils includes
#include "laser_scan_utils/laser_scan.h"

namespace wolf {

class CaptureLaser2D : public CaptureBase
{
    public:
        typedef std::shared_ptr<CaptureLaser2D> Ptr;
    public:
        /** \brief Constructor with ranges
         **/
        CaptureLaser2D(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const std::vector<float>& _ranges);
        virtual ~CaptureLaser2D();
        
        laserscanutils::LaserScan& getScan();

    private:
        SensorLaser2D::Ptr laser_ptr_; //specific pointer to sensor laser 2D object
        laserscanutils::LaserScan scan_;

};

inline laserscanutils::LaserScan& CaptureLaser2D::getScan()
{
    return scan_;
}

} // namespace wolf

#endif /* CAPTURE_LASER_2D_H_ */
