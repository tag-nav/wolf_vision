#include "utils_gtest.h"
#include "base/common/node_base.h"

class CChildBase;

class CParentBase : public wolf::NodeBase
{
   public:

      std::list<std::shared_ptr<CChildBase> > child_list_;

      CParentBase() :
          NodeBase("")
      {};

      virtual ~CParentBase(){};

      virtual void addChild(std::shared_ptr<CChildBase> _child_ptr) final
      {
         child_list_.push_back(_child_ptr);
      }
};

class CParentDerived : public CParentBase
{
   public:

      CParentDerived(){};
};

class CChildBase : public wolf::NodeBase, public std::enable_shared_from_this<CChildBase>
{
   public:
      std::shared_ptr<CParentBase> parent_ptr_;

      CChildBase(std::shared_ptr<CParentBase> _parent_ptr) :
          NodeBase(""),
          parent_ptr_(_parent_ptr)
      {
         auto wptr = std::shared_ptr<CChildBase>( this, [](CChildBase*){} );

         parent_ptr_->addChild(shared_from_this());
      };
};

class CChildDerived : public CChildBase
{
   public:

      CChildDerived(std::shared_ptr<CParentBase> _parent_ptr) : CChildBase(_parent_ptr){};
};

TEST(TestTest, SingleTest)
{
    std::shared_ptr<CParentDerived> parent_derived_ptr = std::make_shared<CParentDerived>();

    std::shared_ptr<CChildDerived> child_derived_ptr = std::make_shared<CChildDerived>(parent_derived_ptr);

    ASSERT_EQ(child_derived_ptr, parent_derived_ptr->child_list_.front());
    ASSERT_EQ(parent_derived_ptr, child_derived_ptr->parent_ptr_);

    std::string parent_name("my booooring father...");
    std::string child_name("my intelligent son!");
    parent_derived_ptr->setName(parent_name);
    child_derived_ptr->setName(child_name);

    ASSERT_EQ(child_derived_ptr->getName(), child_name);
    ASSERT_EQ(parent_derived_ptr->getName(), parent_name);
    ASSERT_EQ(child_derived_ptr->getName(), parent_derived_ptr->child_list_.front()->getName());
    ASSERT_EQ(parent_derived_ptr->getName(), child_derived_ptr->parent_ptr_->getName());

    //std::cout << "parent_derived_ptr->getName() " << parent_derived_ptr->getName() << std::endl;
    //std::cout << "child_derived_ptr->getName() " << child_derived_ptr->getName() << std::endl;
    //std::cout << "child_derived_ptr->parent_ptr_->getName() " << child_derived_ptr->parent_ptr_->getName() << std::endl;
    //std::cout << "parent_derived_ptr->child_list_.front()->getName() " << parent_derived_ptr->child_list_.front()->getName() << std::endl;

//  PRINTF("All good at TestTest::DummyTestExample !\n");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
