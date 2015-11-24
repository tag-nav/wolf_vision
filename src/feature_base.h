#ifndef FEATURE_BASE_H_
#define FEATURE_BASE_H_

// Forward declarations for node templates
class CaptureBase;
class ConstraintBase;

//std includes

//Wolf includes
#include "wolf.h"
#include "node_linked.h"
#include "capture_base.h"
#include "constraint_base.h"

//class FeatureBase
class FeatureBase : public NodeLinked<CaptureBase,ConstraintBase>
{
    protected:
        Eigen::VectorXs measurement_;                   ///< Reference to the measurement
        Eigen::MatrixXs measurement_covariance_;        ///< Reference to the measurement covariance
        std::list<ConstraintBase*> constraint_to_list_;   ///< List of constraints linked TO this feature
        
    public:
        /** \brief Constructor from capture pointer and measure dim
         * 
         * \param _dim_measurement the dimension of the measurement space
         * 
         */
        FeatureBase(unsigned int _dim_measurement);

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
        FeatureBase(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);

        /** \brief Destructor
         *
         * Destructor
         *
         */
        virtual ~FeatureBase();

        /** \brief Adds a constraint from this feature (as a down node)
         *
         * Adds a constraint from this feature (as a down node)
         *
         */
        void addConstraintFrom(ConstraintBase* _co_ptr);

        /** \brief Adds a constraint to this feature (down node from other feature)
         *
         * Adds a constraint to this feature (down node from other feature)
         *
         */
        void addConstraintTo(ConstraintBase* _co_ptr);

        /** \brief Remove a constraint to this feature
         *
         * Remove a constraint to this feature
         *
         **/
        void removeConstraintTo(ConstraintBase* _ctr_ptr);

        /** \brief Gets the number of constraints linked with this frame
         *
         * Gets the number of constraints linked with this frame
         *
         **/
        unsigned int getHits() const;

        /** \brief Gets the list of constraints linked with this frame
         *
         * Gets the list of constraints linked with this frame
         *
         **/
        std::list<ConstraintBase*>* getConstraintToListPtr();

        /** \brief Gets the capture pointer
         *
         * Gets the capture pointer
         *
         */
        CaptureBase* getCapturePtr() const;

        /** \brief Gets the frame pointer
         *
         * Gets the frame pointer
         *
         */
        FrameBase* getFramePtr() const;

        /** \brief Gets the constraint list (down nodes) pointer
         *
         * Gets the constraint list (down nodes) pointer
         *
         */
        ConstraintBaseList* getConstraintFromListPtr();
        
        void getConstraintFromList(ConstraintBaseList & _ctr_list);

        const Eigen::VectorXs & getMeasurement() const;
        
        /** \brief Returns _ii component of measurement vector
         * 
         * Returns _ii component of measurement_ vector
         * WARNING: To be fast, it does not check that index _ii is smaller than dimension.
         * 
         **/
        WolfScalar getMeasurement(unsigned int _ii) const;

        const Eigen::MatrixXs & getMeasurementCovariance() const;

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
