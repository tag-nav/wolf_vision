#ifndef NODE_BASE_H_
#define NODE_BASE_H_

// Wolf includes
#include "wolf.h"


namespace wolf {

/** \brief Base class for Nodes
 *
 * Base class for all Nodes in the Wolf tree. Each node has
 *
 *  - A unique ID. The class implements the ID factory.
 *
 *  - A unique category name, strictly within this range of possibilities:
 *    - "BASE" -- should not be used
 *    - "PROBLEM"
 *    - "HARDWARE"
 *    - "SENSOR"
 *    - "PROCESSOR"
 *    - "TRAJECTORY"
 *    - "FRAME"
 *    - "CAPTURE"
 *    - "FEATURE"
 *    - "CONSTRAINT"
 *    - "MAP"
 *    - "LANDMARK"
 *
 *  - A unique type, which is a subcategory of the above. The list here cannot be exhaustive, but a few examples follow:
 *    - "Camera"
 *    - "LIDAR 2D"
 *    - "Point 3D"
 *    - "Lidar 2D processor"
 *
 *    please refer to each base class derived from NodeLinked for better examples of their types.
 *
 *  - A name, defined in each application, which is specific of each object. Again, just some examples:
 *    - "Front-left camera"
 *    - "Main wheel odometer"
 *    - "IMU delta pre-integrator processor"
 *
 *    These names are only required for objects being defined by the user at configuration time, such as:
 *    - Sensors
 *    - Processors
 *
 *    Additionally, but not necessarily, one could also give names to:
 *    - Pre-defined maps or landmarks (provided to the problem)
 *    - Each of the generated captures --> these names could, for example, be a copy of the sensor name.
 *
 *    Finally, all the other node names in the Wolf Tree are not required, and in fact they are not used for anything.
 *
 **/

class NodeBase
{
    private:
        static unsigned int node_id_count_; ///< Object counter (acts as simple ID factory)

        struct Serializer;

    protected:
        ProblemWPtr problem_ptr_;

        unsigned int node_id_;   ///< Node id. It is unique over the whole Wolf Tree
        std::string node_category_; ///< Text label identifying the category of node ("SENSOR", "FEATURE", etc)
        std::string node_type_;  ///< Text label identifying the type or subcategory of node ("Pin Hole", "Point 2D", etc)
        std::string node_name_;  ///< Text label identifying each specific object ("left camera", "LIDAR 1", "PointGrey", "Andrew", etc)

        bool is_removing_; ///< A flag for safely removing nodes from the Wolf tree. See remove().

    public: 

        NodeBase(const std::string& _category, const std::string& _type = "Undefined", const std::string& _name = "");
        virtual ~NodeBase() = default;

        unsigned int nodeId()  const;
        std::string getCategory() const;
        std::string getType()  const;
        std::string getName()  const;
        bool isRemoving() const;

        void setType(const std::string& _type);
        void setName(const std::string& _name);

        ProblemPtr getProblem() const;
        virtual void setProblem(ProblemPtr _prob_ptr);
};

} // namespace wolf

#include <iostream>

namespace wolf{

inline NodeBase::NodeBase(const std::string& _category, const std::string& _type, const std::string& _name) :
        problem_ptr_(), // nullptr
        node_id_(++node_id_count_),
        node_category_(_category),
        node_type_(_type),
        node_name_(_name),
        is_removing_(false)
{
    //
}

inline unsigned int NodeBase::nodeId() const
{
    return node_id_;
}

inline std::string NodeBase::getCategory() const
{
    return node_category_;
}

inline std::string NodeBase::getType() const
{
    return node_type_;
}

inline std::string NodeBase::getName() const
{
    return node_name_;
}

inline bool NodeBase::isRemoving() const
{
    return is_removing_;
}

inline void NodeBase::setType(const std::string& _type)
{
    node_type_ = _type;
}

inline void NodeBase::setName(const std::string& _name)
{
    node_name_ = _name;
}

inline ProblemPtr NodeBase::getProblem() const
{
    return problem_ptr_.lock();
}

inline void NodeBase::setProblem(ProblemPtr _prob_ptr)
{
    problem_ptr_ = _prob_ptr;
}

} // namespace wolf

#endif /* NODE_BASE_H_ */
