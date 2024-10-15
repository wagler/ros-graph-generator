// STL includes
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <fcntl.h>
#include <regex>
#include <utility>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Protobuf message includes
#include "graph.pb.h"
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

// Local Includes
#include "DummyNode.hpp"

using namespace std::chrono_literals;

DummyNode::DummyNode(const ros_graph::Node &node, const std::unordered_map<std::string, uint32_t> &topics)
    : Node(node.name().c_str())
{

    this->node = node;
    this->topics = topics;
    // Do the actual setup of the ROS structures
    int numErrors = 0;
    numErrors += setupTimers();
    numErrors += setupCallbacks();
    numErrors += setupCallbackGroups();
    numErrors += setupSubscriptions();

    if (numErrors > 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Node is not well formed. Exiting...");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Node %s is started.", node.name().c_str());
}

int DummyNode::setupCallbackGroups()
{
    int numErrors = 0;
    for (const auto &cbGroup : node.callback_groups())
    {
        std::set<std::string> realMembers;

        // Verify the members listed in the callback group are all real callbacks registered in the node
        for (const auto &member : cbGroup.members())
        {
            if (callbacks.find(member) == callbacks.end())
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Callback group %s contains non-existant member callback %s", cbGroup.name().c_str(), member.c_str());
                numErrors++;
                continue;
            }
            realMembers.insert(member);
        }

        // Create the actual callback group object with ROS
        rclcpp::CallbackGroup::SharedPtr temp_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Add this callback group to the list of callback groups in this node. We'll use it later in the subscription setup.
        callbackGroups.push_back(std::make_pair(temp_callback_group, realMembers));
    }

    if (numErrors > 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Detected %d errors with the callback groups.", numErrors);
    }
    return numErrors;
}

int DummyNode::setupCallbackHelper(const ros_graph::Callback &cb)
{
    int numErrors = 0;
    DummyNode::Callback tempCB;

    tempCB.execTimeUs = cb.exec_time_us();

    for (const auto &topic : cb.publishes_to())
    {
        if (topics.find(topic) == topics.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: %s publishes to a non-existant topic %s", cb.name().c_str(), topic.c_str());
            numErrors++;
            continue;
        }

        // If we don't have a publisher for this topic yet, then create it
        if (publishers.find(topic) == publishers.end())
        {
            // TODO: fix the message type for the topic
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr newPublisher =
                this->create_publisher<std_msgs::msg::String>(topic, 10);

            // Insert a pointer to the publisher into our bookkeeping structure.
            publishers.insert(std::make_pair(topic, newPublisher));
        }

        tempCB.publishesTo.push_back(topic);
    }

    callbacks.insert(std::make_pair(cb.name(), tempCB));

    return numErrors;
}

int DummyNode::setupTimers()
{
    int numErrors = 0;
    for (const auto &timer : node.timers())
    {
        // Extract the timer's frequency and calculate its period
        const uint32_t freq_hz = timer.frequency_hz();
        const double period = 1000 / ((double)freq_hz);
        const auto timer_duration = std::chrono::milliseconds(static_cast<int64_t>(std::round(period)));
        const std::string &name = timer.callback().name();

        numErrors += setupCallbackHelper(timer.callback());

        // Check if this timer is part of a callback group
        rclcpp::CallbackGroup::SharedPtr rosCbGroup = nullptr;
        for (const auto &cbGroup : callbackGroups)
        {
            const std::set<std::string> &members = cbGroup.second;
            if (members.find(name) != members.end())
            {
                rosCbGroup = cbGroup.first;
            }
        }

        rclcpp::SubscriptionOptions options;
        options.callback_group = rosCbGroup;
        // Create the actual timer
        RCLCPP_INFO(this->get_logger(), "Setup timer %s with frequency %u Hz, period is %ld ms", name.c_str(), freq_hz, (int64_t)timer_duration.count());
        rclcpp::TimerBase::SharedPtr tempTimer = this->create_wall_timer(
            timer_duration,
            [this, timer, name]() -> void
            { this->execCallback(name, this->callbacks.at(name)); },
            rosCbGroup);
        timers.insert(std::make_pair(name, tempTimer));
    }

    if (numErrors > 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Detected %d errors with the timers.", numErrors);
    }
    return numErrors;
}

int DummyNode::setupCallbacks()
{
    int numErrors = 0;
    for (const auto &callback : node.callbacks())
    {
        numErrors += setupCallbackHelper(callback);
    }

    if (numErrors > 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Detected %d errors with the callbacks.", numErrors);
    }
    return numErrors;
}

int DummyNode::setupSubscriptions()
{
    int numErrors = 0;
    for (const auto &subscription : node.subscriptions())
    {
        bool error = false;
        if (publishers.find(subscription.topic()) == publishers.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: subscription (%s, %s) references a non-existant topic %s", subscription.topic().c_str(), subscription.callback().c_str(), subscription.topic().c_str());
            numErrors++;
            error = true;
        }

        if (callbacks.find(subscription.callback()) == callbacks.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: subscription (%s, %s) references a non-existant callback %s", subscription.topic().c_str(), subscription.callback().c_str(), subscription.callback().c_str());
            numErrors++;
            error = true;
        }

        if (error)
        {
            continue;
        }

        // If the subscription was valid, then set it up in ROS
        const std::string topic = subscription.topic();
        const std::string callbackName = subscription.callback();

        // Check if this timer is part of a callback group
        rclcpp::CallbackGroup::SharedPtr rosCbGroup = nullptr;
        for (const auto &cbGroup : callbackGroups)
        {
            const std::set<std::string> &members = cbGroup.second;
            if (members.find(callbackName) != members.end())
            {
                rosCbGroup = cbGroup.first;
            }
        }
        rclcpp::SubscriptionOptions options;
        options.callback_group = rosCbGroup;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr newSub =
            this->create_subscription<std_msgs::msg::String>(
                topic, 10,
                [this, callbackName](const std_msgs::msg::String &msg) -> void
                {
                    RCLCPP_INFO(this->get_logger(), "Callback '%s' invoked with message '%s'", callbackName.c_str(), msg.data.c_str());
                    this->execCallback(callbackName, callbacks.at(callbackName));
                },
                options);

        subscriptions.push_back(newSub);
    }

    if (numErrors > 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Detected %d errors with the subscriptions.", numErrors);
    }
    return numErrors;
}

void DummyNode::execCallback(const std::string name, const DummyNode::Callback cb)
{
    const auto start = std::chrono::high_resolution_clock::now();
    const auto end = start + std::chrono::microseconds(cb.execTimeUs);

    // Busy-wait loop
    while (std::chrono::high_resolution_clock::now() < end)
    {
    }

    for (const auto &topic : cb.publishesTo)
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! from " + name;
        RCLCPP_INFO(this->get_logger(), "%s publishing: '%s' on topic %s",
                    name.c_str(), message.data.c_str(), topic.c_str());

        publishers.at(topic)->publish(message);
    }
}