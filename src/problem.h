#ifndef PROBLEM_H_
#define PROBLEM_H_

// Fwd refs
namespace wolf{
class HardwareBase;
class TrajectoryBase;
class MapBase;
class ProcessorMotion;
class TimeStamp;
struct IntrinsicsBase;
struct ProcessorParamsBase;
}

//wolf includes
#include "node_base.h"

// std includes
#include <utility> // pair


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
        StateBlock* state_block_ptr_;
        Scalar* scalar_ptr_;
};
struct ConstraintNotification
{
        Notification notification_;
        ConstraintBase* constraint_ptr_;
        unsigned int id_;
};


/** \brief Wolf problem node element in the Wolf Tree
 * 
 * A node has five main data members:
 * - An unique ID to identify it over the whole Wolf Tree (inherited from Node)
 * - A label indicating the node nature (inherited from Node)
 * - An enum indicating tree location (see NodeLocation enum at wolf.h)
 * - down_node_list_: A list of shared pointers to derived node objects, specified by the template parameter LowerType.
 * - up_node_: A regular pointer to a derived node object, specified by the template parameter UpperType.
 *
 */
class Problem : public NodeBase
{
    public:
        typedef NodeBase* LowerNodePtr; // Necessatry for destruct() of node_linked

    protected:
        std::map<std::pair<StateBlock*, StateBlock*>, Eigen::MatrixXs> covariances_;
        NodeLocation location_; // TODO: should it be in node_base?
        TrajectoryBase* trajectory_ptr_;
        MapBase* map_ptr_;
        HardwareBase* hardware_ptr_;
        ProcessorMotion* processor_motion_ptr_;
        StateBlockList state_block_ptr_list_;
        std::list<StateBlockNotification> state_block_notification_list_;
        std::list<ConstraintNotification> constraint_notification_list_;
        bool origin_setted_;

    public:

        /** \brief Constructor from frame structure
         *
         */
        Problem(FrameStructure _frame_structure);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         */
        virtual ~Problem();

        /** \brief Wolf destructor
         *
         * Wolf destructor (please use it instead of delete for guaranteeing the wolf tree integrity)
         *
         */
        virtual void destruct() final;

        /** \brief Sets an origin frame with a covariance
         *
         * Sets an origin frame with a covariance
         *
         */
        virtual void setOrigin(const Eigen::VectorXs& _origin_pose, const Eigen::MatrixXs& _origin_cov,
                               const TimeStamp& _ts);


        /** \brief add sensor to hardware
         * \param _sen_ptr pointer to the sensor to add
         */
        void addSensor(SensorBase* _sen_ptr);

        /** \brief Factory method to install (create and add) sensors only from its properties
         * \param _sen_type type of sensor
         * \param _unique_sensor_name unique sensor name, used to identify the particular instance of the sensor
         * \param _extrinsics a vector of extrinsic parameters: size 2 for 2D position, 3 for 2D pose, 3 for 3D position, 7 for 3D pose.
         * \param _intrinsics a base-pointer to a derived struct defining the intrinsic parameters.
         */
        SensorBase* installSensor(std::string _sen_type, //
                                  std::string _unique_sensor_name, //
                                  const Eigen::VectorXs& _extrinsics, //
                                  IntrinsicsBase* _intrinsics = nullptr);

        /** \brief Factory method to install (create and add) sensors only from its properties -- Helper method loading parameters from file
         * \param _sen_type type of sensor
         * \param _unique_sensor_name unique sensor name, used to identify the particular instance of the sensor
         * \param _extrinsics a vector of extrinsic parameters: size 2 for 2D position, 3 for 2D pose, 3 for 3D position, 7 for 3D pose.
         * \param _intrinsics_filename the name of a file containing the intrinsic parameters in a format compatible with the intrinsics creator registered in IntrinsicsFactory under the key _sen_type.
         */
        SensorBase* installSensor(std::string _sen_type, //
                                  std::string _unique_sensor_name, //
                                  const Eigen::VectorXs& _extrinsics, //
                                  std::string _intrinsics_filename);

        /** \brief Factory method to install (create, and add to sensor) processors only from its properties
         *
         * This method creates a Processor, and adds it to the specified sensor's list of processors
         * \param _prc_type type of processor
         * \param _unique_processor_name unique processor name, used to identify the particular instance of the processor
         * \param _corresponding_sensor_ptr pointer to the sensor where the processor will be installed.
         * \param _prc_params a base-pointer to a derived struct defining the processor parameters.
         */
        ProcessorBase* installProcessor(std::string _prc_type, //
                                        std::string _unique_processor_name, //
                                        SensorBase* _corresponding_sensor_ptr, //
                                        ProcessorParamsBase* _prc_params = nullptr);

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
        void installProcessor(std::string _prc_type, //
                              std::string _unique_processor_name, //
                              std::string _corresponding_sensor_name, //
                              std::string _params_filename = "");

        /** \brief Set the processor motion
         *
         * Set the processor motion. It will provide the state.
         *
         */
        void setProcessorMotion(ProcessorMotion* _processor_motion_ptr);
        void setProcessorMotion(std::string _unique_processor_name);

        /** \brief Create Frame of the correct size
         *
         * This acts as a Frame factory, but also takes care to update related lists in WolfProblem
         */
        FrameBase* createFrame(FrameKeyType _frame_key_type, const TimeStamp& _time_stamp);

        /** \brief Create Frame from vector
         *
         * This acts as a Frame factory, but also takes care to update related lists in WolfProblem
         */
        FrameBase* createFrame(FrameKeyType _frame_key_type, const Eigen::VectorXs& _frame_state,
                               const TimeStamp& _time_stamp);

        /** \brief Get the state at last timestamp (and return timestamp via parameter)
         */
        Eigen::VectorXs getCurrentState();
        Eigen::VectorXs getCurrentState(TimeStamp& _ts);
        void getCurrentState(Eigen::VectorXs& state);
        void getCurrentState(Eigen::VectorXs& state, TimeStamp& _ts);

        /** \brief Get the state at a given timestamp
         */
        Eigen::VectorXs getStateAtTimeStamp(const TimeStamp& _ts);
        void getStateAtTimeStamp(const TimeStamp& _ts, Eigen::VectorXs& state);

        unsigned int getFrameStructureSize();
        Eigen::VectorXs zeroState();

        /** \brief Give the permission to a processor to create a new keyFrame
         */
        bool permitKeyFrame(ProcessorBase* _processor_ptr);

        /** \brief New key frame callback
         *
         * New key frame callback: It should be called by any processor that creates a new keyframe. It calls the keyFrameCallback of the rest of processors.
         */
        void keyFrameCallback(FrameBase* _keyframe_ptr, ProcessorBase* _processor_ptr, const Scalar& _time_tolerance);

        LandmarkBase* addLandmark(LandmarkBase* _lmk_ptr);

        void addLandmarkList(LandmarkBaseList _lmk_list);

        /** \brief Adds a new state block to be added to solver manager
         */
        StateBlock* addStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a new state block to be updated to solver manager
         */
        void updateStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a state block to be removed to solver manager
         */
        void removeStateBlockPtr(StateBlock* _state_ptr);

        /** \brief Adds a new constraint to be added to solver manager
         */
        ConstraintBase* addConstraintPtr(ConstraintBase* _constraint_ptr);

        /** \brief Adds a constraint to be removed to solver manager
         */
        void removeConstraintPtr(ConstraintBase* _constraint_ptr);

        /** \brief Clear covariance
         */
        void clearCovariance();

        /** \brief Adds a new covariance block
         */
        void addCovarianceBlock(StateBlock* _state1, StateBlock* _state2, const Eigen::MatrixXs& _cov);

        /** \brief Gets a covariance block
         */
        bool getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov, const int _row = 0,
                                const int _col=0);

        /** \brief Gets the covariance of a frame
         */
        bool getFrameCovariance(FrameBase* _frame_ptr, Eigen::MatrixXs& _covariance);
        Eigen::MatrixXs getFrameCovariance(FrameBase* _frame_ptr);

        /** \brief Gets the covariance of a landmark
         */
        bool getLandmarkCovariance(LandmarkBase* _landmark_ptr, Eigen::MatrixXs& _covariance);
        Eigen::MatrixXs getLandmarkCovariance(LandmarkBase* _landmark_ptr);

        /** \brief Adds a map
         */
        MapBase* addMap(MapBase* _map_ptr);

        /** \brief Adds a trajectory
         */
        TrajectoryBase* addTrajectory(TrajectoryBase* _trajectory_ptr);

        /** \brief Gets a pointer to map
         */
        MapBase* getMapPtr();

        /** \brief Gets a pointer to trajectory
         */
        TrajectoryBase* getTrajectoryPtr();

        /** \brief Gets a pointer to Hardware
         */
        HardwareBase* getHardwarePtr();

        /** \brief Returns a pointer to last frame
         **/
        FrameBase* getLastFramePtr();

        /** \brief Returns a pointer to last key frame
         */
        FrameBase* getLastKeyFramePtr();

        /** \brief Gets a pointer to the state units list
         */
        StateBlockList* getStateListPtr();

        /** \brief Gets a queue of state blocks notification to be handled by the solver
         */
        std::list<StateBlockNotification>& getStateBlockNotificationList();

        /** \brief Gets a queue of constraint notification to be handled by the solver
         */
        std::list<ConstraintNotification>& getConstraintNotificationList();

        /** \brief get top node (this)
         */
        Problem* getTop();

        /** \brief get this node
         */
        Problem* getProblem();

        /** \brief Returns a true (is top)
         */
        bool isTop();

        /** \brief Remove Down Node
         *
         * This empty function is needed by the destruct() node_linked function.
         */
        void removeDownNode(const LowerNodePtr _ptr){};

        /** \brief get a sensor pointer by its name
         * \param _sensor_name The sensor name, as it was installed with installSensor()
         */
        SensorBase* getSensorPtr(const std::string& _sensor_name);

};

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

inline Problem* Problem::getProblem()
{
    return this;
}

inline Problem* Problem::getTop()
{
    return this;
}

inline bool Problem::isTop()
{
    return true;
}

} // namespace wolf


#endif // PROBLEM_H
