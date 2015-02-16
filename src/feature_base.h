#ifndef FEATURE_BASE_H_
#define FEATURE_BASE_H_

// Forward declarations for node templates
class CaptureBase;
class CorrespondenceBase;

//std includes

//Wolf includes
#include "wolf.h"
#include "node_linked.h"
#include "capture_base.h"
#include "correspondence_base.h"

//class FeatureBase
class FeatureBase : public NodeLinked<CaptureBase,CorrespondenceBase>
{
    protected:
        Eigen::VectorXs measurement_;
        Eigen::MatrixXs measurement_covariance_; ///< Noise of the measurement
        
    public:
        /** \brief Constructor from capture pointer and measure dim
         * 
         * \param _capt_ptr a pointer to the Capture up node
         * \param _dim_measurement the dimension of the measurement space
         * 
         */
        FeatureBase(const CaptureBasePtr& _capt_ptr, unsigned int _dim_measurement);

        /** \brief Constructor from capture pointer and measure
         *
         * \param _capt_ptr a pointer to the Capture up node
         * \param _measurement the measurement
         *
         */
        // measurement ha d'anar amb covariance, si cal, posem Identity com a default...
        //FeatureBase(const CaptureBasePtr& _capt_ptr, const Eigen::VectorXs& _measurement);

        /** \brief Constructor from capture pointer and measure
         *
         * \param _capt_ptr a shared pointer to the Capture up node
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
        FeatureBase(const CaptureBasePtr& _capt_ptr, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);

        virtual ~FeatureBase();
        
        //void linkToCapture(const CaptureBaseShPtr& _capt_ptr); //JVN: inútil, el constructor ja crea el feature penjant d'una captura
        
        void addCorrespondence(CorrespondenceBaseShPtr& _co_ptr);

        const CaptureBasePtr getCapturePtr() const;

        const FrameBasePtr getFramePtr() const;
        
//         const CorrespondenceBaseList & getCorrespondenceList() const;
        
        CorrespondenceBaseList* getCorrespondenceListPtr();
        
        Eigen::VectorXs * getMeasurementPtr();
        
        Eigen::MatrixXs * getMeasurementCovariancePtr();

        void setMeasurement(const Eigen::VectorXs & _meas);
        
        void setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov);
        
        /** \brief prints object's info
         * 
         * prints object's info
         * 
         **/
        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
        
};
#endif
