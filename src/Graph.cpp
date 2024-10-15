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
#include <sstream>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Protobuf message includes
#include "graph.pb.h"
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

// Local Includes
#include "Graph.hpp"

using namespace std::chrono_literals;

Graph::Graph(const std::string &graphFilePath)
{
    // Validates the input graph file is well formed, and fills in the member structures
    // that keep track of the topics, timers, callbacks, and subscriptions

    // Open the text file
    int fd = open(graphFilePath.c_str(), O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "Failed to open file " << graphFilePath << std::endl;
        return;
    }

    std::unique_ptr<google::protobuf::io::ZeroCopyInputStream> input(
        new google::protobuf::io::FileInputStream(fd));

    // Read the protobuf message from the text file
    if (!google::protobuf::TextFormat::Parse(input.get(), &graph))
    {
        std::cerr << "Protobuf failed to parse graph configuration file." << std::endl;
        return;
    }

    // Creates the map of topics -> message size
    int numErrors = setupTopics();
    if (numErrors > 0)
    {
        std::cerr << "Detected " << numErrors << " errors in the input graph configuration file.\n";
        std::cerr << "Graph is not well formed. Exiting..." << std::endl;
        return;
    }

    std::cout << "No errors found with input graph configuration file. Creating nodes..." << std::endl;

    name = graph.name();

    // Creates the individual nodes, which contain the callbacks, timers, subscriptions
    for (const auto &node : graph.nodes())
    {
        std::shared_ptr<DummyNode> newNode = std::make_shared<DummyNode>(node, topics);
        nodes.push_back(newNode);
    }

    printGraphStats();

    for (const auto &node : nodes)
    {
        executor.add_node(node);
    }
    executor.spin();
}

int Graph::setupTopics()
{
    int numErrors = 0;
    std::regex pattern(R"((?=.*[A-z0-9_]$)^[/~A-z][A-z0-9_/]*$)");

    // Validate the topic names are all valid
    for (const auto &topic : graph.topics())
    {
        const std::string &name = topic.name();
        if (!std::regex_match(name, pattern))
        {
            std::cerr << "Error: Topic name " << name << " is invalid." << std::endl;
            numErrors++;
            continue;
        }

        const uint32_t msgSize = topic.msg_size_bytes();
        topics.insert(std::make_pair(name, msgSize));
    }

    if (numErrors > 0)
    {
        std::cerr << "Error: detected " << numErrors << " errors with the topics" << std::endl;
    }

    return numErrors;
}

/*
static inline printTopic(std::stringstream &ss, const ros_graph::Topic &topic)
{
    uint32_t count = 1;
    for (const auto &topic : graph.topics())
    {
        std::cout << "\t" << count << ") " << topic.name() << "\n";
        std::cout << "\t\tSize (B)" << topic.msg_size_bytes() << "\n";
        count++;
    }
}

static inline printNode(const ros_graph::Node &node)
{
}

static inline printTimer(const ros_graph::Timer &timer)
{
}

static inline printCallback(const ros_graph::Callback &callback)
{
}

static inline printSubscription(const ros_graph::Subscription &subscription)
{
}

static inline printCallbackGroup(const ros_graph::CallbackGroup &callbackGroup)
{
}

void Graph::printGraphStats() const
{
    std::cout << "Graph Name: " << graph.name() << "\n";

    std::cout << "Topics:\n";
    uint32_t count = 1;
    for (const auto &topic : graph.topics())
    {
        std::cout << "\t" << count << ") " << topic.name() << "\n";
        std::cout << "\t\tSize (B)" << topic.msg_size_bytes() << "\n";
        count++;
    }

    std::cout << "Nodes:\n";
    uint32_t count = 1;
    for (const auto &node : graph.nodes())
    {
        std::cout << "\t" << count << ") " << node.name() << "\n";
        std::cout << "\t\tSize (B)" << topic.msg_size_bytes() << "\n";
        count++;
    }

    count = 1;
    std::cout << "Timers: \n";
    for (const auto &timer : graph.timers())
    {
        std::cout << "\t" << count << ") " << timer.name() << "\n";
        std::cout << "\t\tFrequency (Hz): " << timer.frequency_hz() << "\n";
        std::cout << "\t\tExecution Time (us): " << timer.exec_time_us() << " us\n";
        std::cout << "\t\tPublishes to topics:\n";
        std::string roman = "i";
        for (const auto &topic : timer.publishes_to())
        {
            std::cout << "\t\t\t" << roman << ") " << topic << "\n";
            roman.append("i");
        }
    }

    count = 1;
    std::cout << "Callbacks:\n";
    for (const auto &callback : graph.callbacks())
    {
        std::cout << "\t" << count << ") " << callback.name() << "\n";
        std::cout << "\t\tExecution Time (us): " << callback.exec_time_us() << " us\n";
        std::cout << "\t\tPublishes to topics:\n";
        std::string roman = "i";
        for (const auto &topic : callback.publishes_to())
        {
            std::cout << "\t\t\t" << roman << ") " << topic << "\n";
            roman.append("i");
        }
    }

    count = 1;
    std::cout << "Subscriptions: \n";
    for (const auto &sub : graph.subscriptions())
    {
        std::cout << "\t" << count << ") " << sub.topic()
                  << " --- " << sub.latency_us() << " us ---> " << sub.callback() << "\n";
    }
}
*/

void Graph::printGraphStats() const
{
    std::cout << "Graph Name: " << graph.name() << "\n";

    // Print Nodes
    std::cout << "Nodes:\n";
    for (const auto &node : graph.nodes())
    {
        std::cout << "  Node Name: " << node.name() << "\n";

        // Print Timers
        std::cout << "  Timers:\n";
        for (const auto &timer : node.timers())
        {
            std::cout << "    Frequency (Hz): " << timer.frequency_hz() << "\n";
            std::cout << "    Callback Name: " << timer.callback().name() << "\n";
        }

        // Print Callbacks
        std::cout << "  Callbacks:\n";
        for (const auto &callback : node.callbacks())
        {
            std::cout << "    Name: " << callback.name() << "\n";
            std::cout << "    Execution Time (us): " << callback.exec_time_us() << "\n";
            std::cout << "    Publishes To: ";
            for (const auto &topic : callback.publishes_to())
            {
                std::cout << topic << " ";
            }
            std::cout << "\n";
        }

        // Print Subscriptions
        std::cout << "  Subscriptions:\n";
        for (const auto &subscription : node.subscriptions())
        {
            std::cout << "    Callback: " << subscription.callback() << "\n";
            std::cout << "    Topic: " << subscription.topic() << "\n";
            std::cout << "    Latency (us): " << subscription.latency_us() << "\n";
        }

        // Print Callback Groups
        std::cout << "  Callback Groups:\n";
        for (const auto &group : node.callback_groups())
        {
            std::cout << "    Group Name: " << group.name() << "\n";
            std::cout << "    Members: ";
            for (const auto &member : group.members())
            {
                std::cout << member << " ";
            }
            std::cout << "\n";
        }
    }

    // Print Topics
    std::cout << "Topics:\n";
    for (const auto &topic : graph.topics())
    {
        std::cout << "  Topic Name: " << topic.name() << "\n";
        std::cout << "  Message Size (bytes): " << topic.msg_size_bytes() << "\n";
    }
}