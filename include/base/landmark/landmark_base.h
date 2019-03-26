#ifndef LANDMARK_BASE_H_
#define LANDMARK_BASE_H_

// Fwd references
namespace wolf{
class MapBase;
class StateBlock;
}

//Wolf includes
#include "base/wolf.h"
#include "base/node_base.h"
#include "base/time_stamp.h"

//std includes

// yaml
#include "yaml-cpp/yaml.h"

namespace wolf {

//class LandmarkBase
class LandmarkBase : public NodeBase, public std::enable_shared_from_this<LandmarkBase>
{
    private:
        MapBaseWPtr map_ptr_;
        FactorBasePtrList constrained_by_list_;
        std::vector<StateBlockPtr> state_block_vec_; ///< vector of state blocks, in the order P, O.

        static unsigned int landmark_id_count_;

    protected:
        unsigned int landmark_id_; ///< landmark unique id
        TimeStamp stamp_;       ///< stamp of the creation of the landmark
        Eigen::VectorXs descriptor_;    //TODO: agree? JS: No: It is not general enough as descriptor to be in LmkBase.

    public:

        /** \brief Constructor with type, time stamp and the position state pointer (optional orientation state pointer)
         *
         * Constructor with type, and state pointer
         * \param _tp indicates landmark type.(types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the position
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
         *
         **/
    LandmarkBase(const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr = nullptr);
    LandmarkBase(MapBaseWPtr _ptr, const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr = nullptr);
        virtual ~LandmarkBase();
        virtual void remove();
        virtual YAML::Node saveToYaml() const;

        // Properties
        unsigned int id();
        void setId(unsigned int _id);

        // Fix / unfix
        void fix();
        void unfix();
        bool isFixed() const;

        // State blocks
        const std::vector<StateBlockPtr>& getStateBlockVec() const;
        std::vector<StateBlockPtr>& getStateBlockVec();
        std::vector<StateBlockPtr> getUsedStateBlockVec() const;
        StateBlockPtr getStateBlockPtr(unsigned int _i) const;
        void setStateBlockPtr(unsigned int _i, StateBlockPtr _sb_ptr);
        StateBlockPtr getP() const;
        StateBlockPtr getO() const;
        void setPPtr(const StateBlockPtr _p_ptr);
        void setOPtr(const StateBlockPtr _o_ptr);
        virtual void registerNewStateBlocks();
        Eigen::VectorXs getState() const;
        void getState(Eigen::VectorXs& _state) const;
        bool getCovariance(Eigen::MatrixXs& _cov) const;
        Eigen::MatrixXs getCovariance() const;

    protected:
        virtual void removeStateBlocks();

        // Descriptor
    public:
        const Eigen::VectorXs& getDescriptor() const;
        Scalar getDescriptor(unsigned int _ii) const;
        void setDescriptor(const Eigen::VectorXs& _descriptor);

        // Navigate wolf tree
        virtual void setProblem(ProblemPtr _problem) final;

        FactorBasePtr addConstrainedBy(FactorBasePtr _ctr_ptr);
        unsigned int getHits() const;
        FactorBasePtrList& getConstrainedByList();

        void setMapPtr(const MapBasePtr _map_ptr);
        MapBasePtr getMap();
        void link(MapBasePtr);
        template<typename classType, typename... T>
        static std::shared_ptr<LandmarkBase> emplace(MapBasePtr _map_ptr, T&&... all);

};

}

#include "base/map_base.h"
#include "base/factor/factor_base.h"
#include "base/state_block.h"

namespace wolf{

template<typename classType, typename... T>
std::shared_ptr<LandmarkBase> LandmarkBase::emplace(MapBasePtr _map_ptr, T&&... all)
{
    LandmarkBasePtr lmk = std::make_shared<classType>(std::forward<T>(all)...);
    lmk->link(_map_ptr);
    return lmk;
}
inline void LandmarkBase::setProblem(ProblemPtr _problem)
{
    NodeBase::setProblem(_problem);
}

inline MapBasePtr LandmarkBase::getMap()
{
    return map_ptr_.lock();
}

inline void LandmarkBase::setMapPtr(const MapBasePtr _map_ptr)
{
    map_ptr_ = _map_ptr;
}

inline unsigned int LandmarkBase::id()
{
    return landmark_id_;
}

inline void LandmarkBase::setId(unsigned int _id)
{
    landmark_id_ = _id;
    if (_id > landmark_id_count_)
        landmark_id_count_ = _id;
}

inline unsigned int LandmarkBase::getHits() const
{
    return constrained_by_list_.size();
}

inline FactorBasePtrList& LandmarkBase::getConstrainedByList()
{
    return constrained_by_list_;
}

inline const std::vector<StateBlockPtr>& LandmarkBase::getStateBlockVec() const
{
    return state_block_vec_;
}

inline std::vector<StateBlockPtr>& LandmarkBase::getStateBlockVec()
{
    return state_block_vec_;
}

inline StateBlockPtr LandmarkBase::getStateBlockPtr(unsigned int _i) const
{
    assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
    return state_block_vec_[_i];
}

inline void LandmarkBase::setStateBlockPtr(unsigned int _i, StateBlockPtr _sb_ptr)
{
    state_block_vec_[_i] = _sb_ptr;
}

inline StateBlockPtr LandmarkBase::getP() const
{
    return getStateBlockPtr(0);
}

inline StateBlockPtr LandmarkBase::getO() const
{
    return getStateBlockPtr(1);
}

inline void LandmarkBase::setPPtr(const StateBlockPtr _st_ptr)
{
    setStateBlockPtr(0, _st_ptr);
}

inline void LandmarkBase::setOPtr(const StateBlockPtr _st_ptr)
{
    setStateBlockPtr(1, _st_ptr);
}

inline void LandmarkBase::setDescriptor(const Eigen::VectorXs& _descriptor)
{
    descriptor_ = _descriptor;
}

inline Scalar LandmarkBase::getDescriptor(unsigned int _ii) const
{
    assert(_ii < descriptor_.size() && "LandmarkBase::getDescriptor: bad index");
    return descriptor_(_ii);
}

inline const Eigen::VectorXs& LandmarkBase::getDescriptor() const
{
    return descriptor_;
}

} // namespace wolf
#endif
