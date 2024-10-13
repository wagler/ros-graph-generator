#pragma once

// STL includes
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Protobuf includes
#include "graph.pb.h"
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

using namespace std::chrono_literals;

class Graph : public rclcpp::Node
{
public:
    explicit Graph(const std::string &graphFilePath);
    Graph() = delete;
    Graph(const Graph &) = delete;
    Graph(const Graph &&) = delete;
    Graph &operator=(const Graph &) = delete;
    Graph &operator=(Graph &&) = delete;

    ~Graph()
    {
        // Cleanup protobuf
        google::protobuf::ShutdownProtobufLibrary();
    }

    // Print info on the topics, timers, callbacks, and subscriptions detected in the config file
    void printGraphStats() const;

private:
    // Setup the graph bookkeeping, and do validation of these components to make sure
    // the user-supplied graph is well formed
    int setupTopics();
    int setupTimers();
    int setupCallbacks();
    int setupSubscriptions();

    struct Callback
    {
        std::vector<std::string> publishesTo;
        uint32_t execTimeUs;
    };

    void execCallback(const std::string name, const Callback cb);

    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions;
    std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> timers;
    std::unordered_map<std::string, Callback> callbacks;
    ros_graph::ROSGraph rosGraph;
};
