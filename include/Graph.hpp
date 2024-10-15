#pragma once

// STL includes
#include <string>
#include <vector>
#include <unordered_map>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Protobuf includes
#include "graph.pb.h"
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "DummyNode.hpp"

using namespace std::chrono_literals;

class Graph
{
public:
    explicit Graph(const std::string &graphFilePath);
    Graph() = delete;
    Graph(const Graph &) = delete;
    Graph(const Graph &&) = delete;
    Graph &operator=(const Graph &) = delete;
    Graph &operator=(Graph &&) = delete;

    // Creates the map of topics -> message size
    int setupTopics();

    // Print info on the topics, timers, callbacks, and subscriptions detected in the config file
    void printGraphStats() const;

private:
    std::string name;
    std::vector<DummyNode::SharedPtr> nodes;
    std::unordered_map<std::string, uint32_t> topics;
    ros_graph::Graph graph;
    rclcpp::executors::SingleThreadedExecutor executor;
};
