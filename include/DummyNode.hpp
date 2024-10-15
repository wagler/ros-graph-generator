#pragma once

// STL includes
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Protobuf includes
#include "graph.pb.h"
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

class DummyNode : public rclcpp::Node
{
public:
    explicit DummyNode(const ros_graph::Node &node, const std::unordered_map<std::string, uint32_t> &topics);
    DummyNode() = delete;
    DummyNode(const DummyNode &) = delete;
    DummyNode(const DummyNode &&) = delete;
    DummyNode &operator=(const DummyNode &) = delete;
    DummyNode &operator=(DummyNode &&) = delete;

private:
    // Setup the graph bookkeeping, and do validation of these components to make sure
    // the user-supplied graph is well formed
    int setupTimers();
    int setupCallbacks();
    int setupSubscriptions();
    int setupCallbackGroups();
    int setupCallbackHelper(const ros_graph::Callback &cb);

    // A representation of a callback that we can use to hold the data we need
    // when feeding the execCallback function for subscription handling
    struct Callback
    {
        std::vector<std::string> publishesTo;
        uint32_t execTimeUs;
    };

    void execCallback(const std::string name, const Callback cb);

    // Should be a 1:1 copy of the topics map in Graph
    std::unordered_map<std::string, uint32_t> topics;

    // Maps a topic name to a publisher object for this node to publish on that topic
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers;

    // A list of all the subscriptions this node has for its callbacks
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions;

    // A list of the ROS timers in this node
    std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> timers;

    // A mapping of callback name to callback object. Node this includes timers from the protobuf message
    std::unordered_map<std::string, Callback> callbacks;

    // List of callback groups. Each group is a pair containing the actual ROS callback object, as well as a list of the member callbacks.
    std::vector<std::pair<rclcpp::CallbackGroup::SharedPtr, std::set<std::string>>> callbackGroups;

    // The original node protobuf object this ROS node is built from
    ros_graph::Node node;
};
