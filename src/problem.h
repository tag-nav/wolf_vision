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
#include "wolf.h"

// std includes


namespace wolf {


enum Notification
{
    ADD,
    REMOVE,
    UPDATE
};
struct StateBlockNotification
{
        Notification notification_;
        StateBlockPtr state_block_ptr_;
        Scalar* scalar_ptr_;
};
struct ConstraintNotification
{
        Notification notification_;
        ConstraintBasePtr constraint_ptr_; // TODO check pointer type
        unsigned int id_;
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
        StateBlockList      state_block_list_;
        std::map<std::pair<StateBlockPtr, StateBlockPtr>, Eigen::MatrixXs> covariances_;
        std::list<StateBlockNotification> state_block_notification_list_;
        std::list<ConstraintNotification> constraint_notification_list_;
        bool origin_is_set_;

    private: // CAUTION: THESE METHODS ARE PRIVATE, DO NOT MAKE THEM PUBLIC !!
        Problem(FrameStructure _frame_structure); // USE create() below !!
        void setup();

    public:
        static ProblemPtr create(FrameStructure _frame_structure); // USE THIS AS A CONSTRUCTOR!
        virtual ~Problem();

        // Properties -----------------------------------------
        Size getFrameStructureSize() const;
        void getFrameStructureSize(Size& _x_size, Size& _cov_size) const;

        // Hardware branch ------------------------------------
        HardwareBasePtr getHardwarePtr();
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
        SensorBasePtr getSensorPtr(const std::string& _sensor_name);


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
         * Set the processor motion. It will provide the state.
         */
        void setProcessorMotion(ProcessorMotionPtr _processor_motion_ptr);
        ProcessorMotionPtr setProcessorMotion(const std::string& _unique_processor_name);
        void clearProcessorMotion();
        ProcessorMotionPtr& getProcessorMotionPtr();


        // Trajectory branch ----------------------------------
        TrajectoryBasePtr getTrajectoryPtr();
        virtual void setOrigin(const Eigen::VectorXs& _origin_pose, const Eigen::MatrixXs& _origin_cov,
                               const TimeStamp& _ts);

        /** \brief Emplace Frame of the correct size
         * \param _frame_key_type Either KEY_FRAME or NON_KEY_FRAME
         * \param _time_stamp Time stamp of the frame
         *
         * This acts as a Frame factory, but also takes care to update related lists in WolfProblem:
         *   - Create a Frame
         *   - Add it to Trajectory
         *   - If it is key-frame, update state-block lists in Problem
         */
        FrameBasePtr emplaceFrame(FrameType _frame_key_type, const TimeStamp& _time_stamp);

        /** \brief Emplace Frame from vector
         * \param _frame_key_type Either KEY_FRAME or NON_KEY_FRAME
         * \param _frame_state State vector; must match the size and format of the chosen frame structure
         * \param _time_stamp Time stamp of the frame
         *
         * This acts as a Frame factory, but also takes care to update related lists in WolfProblem
         *   - Create a Frame
         *   - Add it to Trajectory
         *   - If it is key-frame, update state-block lists in Problem
         */
        FrameBasePtr emplaceFrame(FrameType _frame_key_type, const Eigen::VectorXs& _frame_state,
                               const TimeStamp& _time_stamp);

        /** \brief Emplace frame fron string frame_structure
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
        FrameBasePtr emplaceFrame(const std::string& _frame_structure, FrameType _frame_key_type, const Eigen::VectorXs& _frame_state,
                               const TimeStamp& _time_stamp);

        Eigen::VectorXs getCurrentState();
        Eigen::VectorXs getCurrentState(TimeStamp& _ts);
        void getCurrentState(Eigen::VectorXs& state);
        void getCurrentState(Eigen::VectorXs& state, TimeStamp& _ts);
        Eigen::VectorXs getStateAtTimeStamp(const TimeStamp& _ts);
        void getStateAtTimeStamp(const TimeStamp& _ts, Eigen::VectorXs& state);

        Eigen::VectorXs zeroState();

        /** \brief Give the permission to a processor to create a new keyFrame
         */
        bool permitKeyFrame(ProcessorBasePtr _processor_ptr);

        /** \brief New key frame callback
         *
         * New key frame callback: It should be called by any processor that creates a new keyframe. It calls the keyFrameCallback of the rest of processors.
         */
        void keyFrameCallback(FrameBasePtr _keyframe_ptr, ProcessorBasePtr _processor_ptr, const Scalar& _time_tolerance);

        /** \brief Returns a pointer to last frame
         **/
        FrameBasePtr getLastFramePtr();

        /** \brief Returns a pointer to last key frame
         */
        FrameBasePtr getLastKeyFramePtr();




        // Map branch -----------------------------------------
        MapBasePtr getMapPtr();
        LandmarkBasePtr addLandmark(LandmarkBasePtr _lmk_ptr);
        void addLandmarkList(LandmarkBaseList& _lmk_list);
        void loadMap(const std::string& _filename_dot_yaml);
        void saveMap(const std::string& _filename_dot_yaml, const std::string& _map_name = "Map automatically saved by Wolf");



        // Covariances -------------------------------------------
        void clearCovariance();
        void addCovarianceBlock(StateBlockPtr _state1, StateBlockPtr _state2, const Eigen::MatrixXs& _cov);
        bool getCovarianceBlock(StateBlockPtr _state1, StateBlockPtr _state2, Eigen::MatrixXs& _cov, const int _row = 0,
                                const int _col=0);
        bool getFrameCovariance(FrameBasePtr _frame_ptr, Eigen::MatrixXs& _covariance);
        Eigen::MatrixXs getFrameCovariance(FrameBasePtr _frame_ptr);
        bool getLandmarkCovariance(LandmarkBasePtr _landmark_ptr, Eigen::MatrixXs& _covariance);
        Eigen::MatrixXs getLandmarkCovariance(LandmarkBasePtr _landmark_ptr);


        // Solver management ----------------------------------------

        /** \brief Gets a reference to the state units list
         */
        StateBlockList& getStateBlockList();

        /** \brief Adds a new state block to be added to solver manager
         */
        StateBlockPtr addStateBlock(StateBlockPtr _state_ptr);

        /** \brief Adds a new state block to be updated to solver manager
         */
        void updateStateBlockPtr(StateBlockPtr _state_ptr);

        /** \brief Adds a state block to be removed to solver manager
         */
        void removeStateBlockPtr(StateBlockPtr _state_ptr);

        /** \brief Gets a queue of state blocks notification to be handled by the solver
         */
        std::list<StateBlockNotification>& getStateBlockNotificationList();


        /** \brief Gets a queue of constraint notification to be handled by the solver
         */
        std::list<ConstraintNotification>& getConstraintNotificationList();

        /** \brief Adds a new constraint to be added to solver manager
         */
        ConstraintBasePtr addConstraintPtr(ConstraintBasePtr _constraint_ptr);

        /** \brief Adds a constraint to be removed to solver manager
         */
        void removeConstraintPtr(ConstraintBasePtr _constraint_ptr);

        // Print and check ---------------------------------------
        /**
         * \brief print wolf tree
         * \param depth :        levels to show ( 0: H, T, M : 1: H:S:p, T:F, M:L ; 2: T:F:C ; 3: T:F:C:f ; 4: T:F:C:f:c )
         * \param constr_by:     show constraints pointing to F, f and L.
         * \param metric :       show metric info (status, time stamps, state vectors, measurements)
         * \param state_blocks : show state blocks
         */
        void print(int depth = 4, bool constr_by = false, bool metric = true, bool state_blocks = false);
        bool check(int verbose_level = 0);

};

inline wolf::ProcessorMotionPtr& Problem::getProcessorMotionPtr()
{
    return processor_motion_ptr_;
}

} // namespace wolf

// IMPLEMENTATION

namespace wolf
{

inline std::list<StateBlockNotification>& Problem::getStateBlockNotificationList()
{
    return state_block_notification_list_;
}

inline std::list<ConstraintNotification>& Problem::getConstraintNotificationList()
{
    return constraint_notification_list_;
}


} // namespace wolf


#endif // PROBLEM_H
