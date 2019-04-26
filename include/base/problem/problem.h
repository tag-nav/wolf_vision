#ifndef PROBLEM_H_
#define PROBLEM_H_

// Fwd refs
namespace wolf{
class HardwareBase;
class TrajectoryBase;
class MapBase;
class ProcessorMotion;
class StateBlock;
class TimeStamp;
struct IntrinsicsBase;
struct ProcessorParamsBase;
}

//wolf includes
#include "base/common/wolf.h"
#include "base/frame/frame_base.h"
#include "base/state_block/state_block.h"

// std includes
#include <mutex>

namespace wolf {

enum Notification
{
    ADD,
    REMOVE
};

/** \brief Wolf problem node element in the Wolf Tree
 */
class Problem : public std::enable_shared_from_this<Problem>
{

    protected:
        HardwareBasePtr     hardware_ptr_;
        TrajectoryBasePtr   trajectory_ptr_;
        MapBasePtr          map_ptr_;
        ProcessorMotionPtr  processor_motion_ptr_;
        std::map<std::pair<StateBlockPtr, StateBlockPtr>, Eigen::MatrixXs> covariances_;
        SizeEigen state_size_, state_cov_size_, dim_;
        std::map<FactorBasePtr, Notification> factor_notification_map_;
        std::map<StateBlockPtr, Notification> state_block_notification_map_;
        mutable std::mutex mut_factor_notifications_;
        mutable std::mutex mut_state_block_notifications_;
        mutable std::mutex mut_covariances_;
        bool prior_is_set_;

    private: // CAUTION: THESE METHODS ARE PRIVATE, DO NOT MAKE THEM PUBLIC !!
        Problem(const std::string& _frame_structure); // USE create() below !!
        void setup();

    public:
        static ProblemPtr create(const std::string& _frame_structure); // USE THIS AS A CONSTRUCTOR!
        virtual ~Problem();

        // Properties -----------------------------------------
        SizeEigen getFrameStructureSize() const;
        void getFrameStructureSize(SizeEigen& _x_size, SizeEigen& _cov_size) const;
        SizeEigen getDim() const;

        // Hardware branch ------------------------------------
        HardwareBasePtr getHardware();
        void addSensor(SensorBasePtr _sen_ptr);

        /** \brief Factory method to install (create and add) sensors only from its properties
         * \param _sen_type type of sensor
         * \param _unique_sensor_name unique sensor name, used to identify the particular instance of the sensor
         * \param _extrinsics a vector of extrinsic parameters: size 2 for 2D position, 3 for 2D pose, 3 for 3D position, 7 for 3D pose.
         * \param _intrinsics a base-pointer to a derived struct defining the intrinsic parameters.
         */
        SensorBasePtr installSensor(const std::string& _sen_type, //
                                    const std::string& _unique_sensor_name, //
                                    const Eigen::VectorXs& _extrinsics, //
                                    IntrinsicsBasePtr _intrinsics = nullptr);

        /** \brief Factory method to install (create and add) sensors only from its properties -- Helper method loading parameters from file
         * \param _sen_type type of sensor
         * \param _unique_sensor_name unique sensor name, used to identify the particular instance of the sensor
         * \param _extrinsics a vector of extrinsic parameters: size 2 for 2D position, 3 for 2D pose, 3 for 3D position, 7 for 3D pose.
         * \param _intrinsics_filename the name of a file containing the intrinsic parameters in a format compatible with the intrinsics creator registered in IntrinsicsFactory under the key _sen_type.
         */
        SensorBasePtr installSensor(const std::string& _sen_type, //
                                    const std::string& _unique_sensor_name, //
                                    const Eigen::VectorXs& _extrinsics, //
                                    const std::string& _intrinsics_filename);

        /** \brief get a sensor pointer by its name
         * \param _sensor_name The sensor name, as it was installed with installSensor()
         */
        SensorBasePtr getSensor(const std::string& _sensor_name);

        /** \brief Factory method to install (create, and add to sensor) processors only from its properties
         *
         * This method creates a Processor, and adds it to the specified sensor's list of processors
         * \param _prc_type type of processor
         * \param _unique_processor_name unique processor name, used to identify the particular instance of the processor
         * \param _corresponding_sensor_ptr pointer to the sensor where the processor will be installed.
         * \param _prc_params a base-pointer to a derived struct defining the processor parameters.
         */
        ProcessorBasePtr installProcessor(const std::string& _prc_type, //
                                          const std::string& _unique_processor_name, //
                                          SensorBasePtr _corresponding_sensor_ptr, //
                                          ProcessorParamsBasePtr _prc_params = nullptr);

        /** \brief Factory method to install (create, and add to sensor) processors only from its properties
         *
         * This method creates a Processor, and adds it to the specified sensor's list of processors
         *
         * This method is a helper wrapper around the version accepting a sensor pointer instead of a sensor name.
         * \param _prc_type type of processor
         * \param _unique_processor_name unique processor name, used to identify the particular instance of the processor
         * \param _corresponding_sensor_name corresponding sensor name, used to bind the processor to the particular instance of the sensor
         * \param _params_filename name of formatted file (xml, yaml, etc) defining the processor parameters.
         */
        ProcessorBasePtr installProcessor(const std::string& _prc_type, //
                                          const std::string& _unique_processor_name, //
                                          const std::string& _corresponding_sensor_name, //
                                          const std::string& _params_filename = "");

        /** \brief Set the processor motion
         *
         * Set the processor motion.
         */
        void setProcessorMotion(ProcessorMotionPtr _processor_motion_ptr);
        ProcessorMotionPtr setProcessorMotion(const std::string& _unique_processor_name);
        void clearProcessorMotion();
        ProcessorMotionPtr& getProcessorMotion();

        // Trajectory branch ----------------------------------
        TrajectoryBasePtr getTrajectory();
        virtual FrameBasePtr setPrior(const Eigen::VectorXs& _prior_state, //
                                      const Eigen::MatrixXs& _prior_cov, //
                                      const TimeStamp& _ts,
                                      const Scalar _time_tolerance);

        /** \brief Emplace frame from string frame_structure
         * \param _frame_structure String indicating the frame structure.
         * \param _frame_key_type Either KEY_FRAME or NON_KEY_FRAME
         * \param _frame_state State vector; must match the size and format of the chosen frame structure
         * \param _time_stamp Time stamp of the frame
         *
         * This acts as a Frame factory, but also takes care to update related lists in WolfProblem:
         *   - Create a Frame
         *   - Add it to Trajectory
         *   - If it is key-frame, update state-block lists in Problem
         */
        FrameBasePtr emplaceFrame(const std::string& _frame_structure, //
                                  FrameType _frame_key_type, //
                                  const Eigen::VectorXs& _frame_state, //
                                  const TimeStamp& _time_stamp);

        /** \brief Emplace frame from string frame_structure without state
         * \param _frame_structure String indicating the frame structure.
         * \param _frame_key_type Either KEY_FRAME or NON_KEY_FRAME
         * \param _time_stamp Time stamp of the frame
         *
         * This acts as a Frame factory, but also takes care to update related lists in WolfProblem:
         *   - Create a Frame
         *   - Add it to Trajectory
         *   - If it is key-frame, update state-block lists in Problem
         */
        FrameBasePtr emplaceFrame(const std::string& _frame_structure, //
                                  FrameType _frame_key_type, //
                                  const TimeStamp& _time_stamp);

        /** \brief Emplace frame from string frame_structure without structure
         * \param _frame_key_type Either KEY_FRAME or NON_KEY_FRAME
         * \param _frame_state State vector; must match the size and format of the chosen frame structure
         * \param _time_stamp Time stamp of the frame
         *
         * This acts as a Frame factory, but also takes care to update related lists in WolfProblem:
         *   - Create a Frame
         *   - Add it to Trajectory
         *   - If it is key-frame, update state-block lists in Problem
         */
        FrameBasePtr emplaceFrame(FrameType _frame_key_type, //
                                  const Eigen::VectorXs& _frame_state, //
                                  const TimeStamp& _time_stamp);

        /** \brief Emplace frame from string frame_structure without structure nor state
         * \param _frame_key_type Either KEY_FRAME or NON_KEY_FRAME
         * \param _time_stamp Time stamp of the frame
         *
         * This acts as a Frame factory, but also takes care to update related lists in WolfProblem:
         *   - Create a Frame
         *   - Add it to Trajectory
         *   - If it is key-frame, update state-block lists in Problem
         */
        FrameBasePtr emplaceFrame(FrameType _frame_key_type, //
                                  const TimeStamp& _time_stamp);

        // Frame getters
        FrameBasePtr    getLastFrame         ( );
        FrameBasePtr    getLastKeyFrame      ( );
        FrameBasePtr    closestKeyFrameToTimeStamp(const TimeStamp& _ts);

        /** \brief Give the permission to a processor to create a new keyFrame
         *
         * This should implement a black list of processors that have forbidden keyframe creation.
         *   - This decision is made at problem level, not at processor configuration level.
         *   - Therefore, if you want to permanently configure a processor for not creating keyframes, see Processor::voting_active_ and its accessors.
         */
        bool permitKeyFrame(ProcessorBasePtr _processor_ptr);

        /** \brief New key frame callback
         *
         * New key frame callback: It should be called by any processor that creates a new keyframe. It calls the keyFrameCallback of the rest of processors.
         */
        void keyFrameCallback(FrameBasePtr _keyframe_ptr, //
                              ProcessorBasePtr _processor_ptr, //
                              const Scalar& _time_tolerance);

        // State getters
        Eigen::VectorXs getCurrentState         ( );
        void            getCurrentState         (Eigen::VectorXs& state);
        void            getCurrentStateAndStamp (Eigen::VectorXs& state, TimeStamp& _ts);
        Eigen::VectorXs getState                (const TimeStamp& _ts);
        void            getState                (const TimeStamp& _ts, Eigen::VectorXs& state);
        // Zero state provider
        Eigen::VectorXs zeroState ( );
        bool priorIsSet() const;

        // Map branch -----------------------------------------
        MapBasePtr getMap();
        LandmarkBasePtr addLandmark(LandmarkBasePtr _lmk_ptr);
        void addLandmarkList(LandmarkBasePtrList& _lmk_list);
        void loadMap(const std::string& _filename_dot_yaml);
        void saveMap(const std::string& _filename_dot_yaml, //
                     const std::string& _map_name = "Map automatically saved by Wolf");

        // Covariances -------------------------------------------
        void clearCovariance();
        void addCovarianceBlock(StateBlockPtr _state1, StateBlockPtr _state2, const Eigen::MatrixXs& _cov);
        void addCovarianceBlock(StateBlockPtr _state1, const Eigen::MatrixXs& _cov);
        bool getCovarianceBlock(StateBlockPtr _state1, StateBlockPtr _state2, Eigen::MatrixXs& _cov, const int _row = 0, const int _col=0);
        bool getCovarianceBlock(std::map<StateBlockPtr, unsigned int> _sb_2_idx, Eigen::MatrixXs& _cov);
        bool getCovarianceBlock(StateBlockPtr _state, Eigen::MatrixXs& _cov, const int _row_and_col = 0);
        bool getFrameCovariance(FrameBaseConstPtr _frame_ptr, Eigen::MatrixXs& _covariance);
        bool getLastKeyFrameCovariance(Eigen::MatrixXs& _covariance);
        bool getLandmarkCovariance(LandmarkBaseConstPtr _landmark_ptr, Eigen::MatrixXs& _covariance);

        // Solver management ----------------------------------------

        /** \brief Notifies a new state block to be added to the solver manager
         */
        StateBlockPtr addStateBlock(StateBlockPtr _state_ptr);

        /** \brief Notifies a state block to be removed from the solver manager
         */
        void removeStateBlock(StateBlockPtr _state_ptr);

        /** \brief Returns the map of factor notification to be handled by the solver (the map stored in this is emptied)
         */
        std::map<StateBlockPtr,Notification> consumeStateBlockNotificationMap();

        /** \brief Notifies a new factor to be added to the solver manager
         */
        FactorBasePtr addFactor(FactorBasePtr _factor_ptr);

        /** \brief Notifies a factor to be removed from the solver manager
         */
        void removeFactor(FactorBasePtr _factor_ptr);

        /** \brief Returns the map of factor notification to be handled by the solver (the map stored in this is emptied)
         */
        std::map<FactorBasePtr, Notification> consumeFactorNotificationMap();

        // Print and check ---------------------------------------
        /**
         * \brief print wolf tree
         * \param depth :        levels to show ( 0: H, T, M : 1: H:S:p, T:F, M:L ; 2: T:F:C ; 3: T:F:C:f ; 4: T:F:C:f:c )
         * \param constr_by:     show factors pointing to F, f and L.
         * \param metric :       show metric info (status, time stamps, state vectors, measurements)
         * \param state_blocks : show state blocks
         */
        void print(int depth = 4, //
                   bool constr_by = false, //
                   bool metric = true, //
                   bool state_blocks = false);
        void print(const std::string& depth, //
                   bool constr_by = false, //
                   bool metric = true, //
                   bool state_blocks = false);
        bool check(int verbose_level = 0);

};

} // namespace wolf

// IMPLEMENTATION

namespace wolf
{

inline bool Problem::priorIsSet() const
{
    return prior_is_set_;
}

inline ProcessorMotionPtr& Problem::getProcessorMotion()
{
    return processor_motion_ptr_;
}

inline std::map<StateBlockPtr,Notification> Problem::consumeStateBlockNotificationMap()
{
    std::lock_guard<std::mutex> lock(mut_state_block_notifications_);
    return std::move(state_block_notification_map_);
}

inline std::map<FactorBasePtr,Notification> Problem::consumeFactorNotificationMap()
{
    std::lock_guard<std::mutex> lock(mut_factor_notifications_);
    return std::move(factor_notification_map_);
}

} // namespace wolf

#endif // PROBLEM_H
