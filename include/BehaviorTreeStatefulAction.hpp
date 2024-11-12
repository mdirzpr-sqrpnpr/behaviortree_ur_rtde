// BehaviorTreeStatefulAction.hpp

#ifndef BEHAVIOR_TREE_STATEFUL_ACTION_HPP
#define BEHAVIOR_TREE_STATEFUL_ACTION_HPP

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"

template <typename DerivedAction>
class BehaviorTreeStatefulAction : public BT::StatefulActionNode
{
public:
    BehaviorTreeStatefulAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    // The onStart, onRunning, and onHalted methods call the derived class's implementations
    BT::NodeStatus onStart() override
    {
        auto derived = static_cast<DerivedAction*>(this);
        return derived->onStartImpl();
    }

    BT::NodeStatus onRunning() override
    {
        auto derived = static_cast<DerivedAction*>(this);
        return derived->onRunningImpl();
    }

    void onHalted() override
    {
        auto derived = static_cast<DerivedAction*>(this);
        derived->onHaltedImpl();
    }
};

#endif // BEHAVIOR_TREE_STATEFUL_ACTION_HPP