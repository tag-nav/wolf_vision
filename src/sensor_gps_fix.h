
#ifndef SENSOR_GPS_FIX_H_
#define SENSOR_GPS_FIX_H_

//wolf includes
#include "sensor_base.h"

// std includes




namespace wolf {

struct IntrinsicsGPSFix : public IntrinsicsBase
{
        // Empty -- it acts only as a typedef for IntrinsicsBase, but allows future extension with parameters
};

class SensorGPSFix : public SensorBase
{
    public:
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _noise noise standard deviation
         * 
         **/
		SensorGPSFix(StateBlock* _p_ptr, StateBlock* _o_ptr, const double& _noise);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/
        virtual ~SensorGPSFix();
        
        /** \brief Returns noise standard deviation
         * 
         * Returns noise standard deviation
         * 
         **/        
        Scalar getNoise() const;
        
    public:
        static SensorBase* create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsBase* _intrinsics);

};

} // namespace wolf

#endif
