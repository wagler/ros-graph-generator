// STL includes
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <fcntl.h>
#include <regex>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Protobuf message includes
#include "gen/graph.pb.h"
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

// Local Includes
#include "Graph.hpp"

using namespace std::chrono_literals;

Graph::Graph(const std::string &graphFilePath)
    : Node("dummy_node")
{
    // Initialize the protobuf library
    GOOGLE_PROTOBUF_VERIFY_VERSION;

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

    // Read the message from the text file
    if (!google::protobuf::TextFormat::Parse(input.get(), &(this->rosGraph)))
    {
        std::cerr << "Failed to parse message." << std::endl;
        return;
    }

    // Do the actual setup of the ROS structures
    int numErrors = 0;

    // Test 1 - validate the topic names are all valid
    numErrors += setupTopics();

    // Test 2 - validate the timers all publish to only valid topics that exist.
    numErrors += setupTimers();

    // Test 3 - validate the callbacks all publish to only valid topics that exist.
    numErrors += setupCallbacks();

    // Test 4 - validate the subscriptions have a valid callback and topic name
    numErrors += setupSubscriptions();

    if (numErrors > 0)
    {
        std::cerr << "Detected " << numErrors << " errors in the input graph configuration file.\n";
        std::cerr << "Graph is not well formed. Exiting..." << std::endl;
        exit(1);
    }

    std::cout << "No errors found with input graph configuration file." << std::endl;

    printGraphStats();
}

int Graph::setupTopics()
{
    int numErrors = 0;
    std::regex pattern(R"((?=.*[A-z0-9_]$)^[/~A-z][A-z0-9_/]*$)");

    // Test 1 - validate the topic names are all valid
    for (const auto &topic : rosGraph.topics())
    {
        const std::string &name = topic.name();
        if (!std::regex_match(name, pattern))
        {
            std::cerr << "Error: Topic name \"" << name << "\" is not valid.\n";
            numErrors++;
            continue;
        }

        // If topic name is valid string, then create a publisher for it.
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tempPublisher =
            this->create_publisher<std_msgs::msg::String>(name, 10);

        // Insert a pointer to the publisher into our bookkeeping structure.
        std::string tempName = name;
        publishers.insert(tempName, tempPublisher);
    }

    return numErrors;
}

int Graph::setupTimers()
{
    int numErrors = 0;
    for (const auto &timer : rosGraph.timers())
    {
        // Extract the timer's frequency and calculate its period
        const uint32_t freq_hz = timer.frequency_hz();
        const double period = 1 / ((double)freq_hz);
        const auto timer_duration = std::chrono::microseconds(period);

        // Find the topics that the timer publishes to which are valid
        Graph::Callback tempCallback;
        tempCallback.execTimeUs = timer.exec_time_us();

        for (const auto &topic : timer.publishes_to())
        {
            if (topics.find(topic) == topics.end())
            {
                std::cerr << "Error: timer \"" << timer.name() << "\" publishes to non-existant topic \"" << topic << "\"\n";
                numErrors++;
                continue;
            }
            tempCallback.publishesTo.push_back(topic);
        }

        callbacks.insert(timer.name(), tempCallback);

        // Create the actual timer
        rclcpp::TimerBase::SharedPtr tempTimer = this->create_wall_timer(
            timer_duration,
            std::bind(&Graph::execCallback, this, timer.name(), callbacks.at(timer.name())));

        // Insert the timer into the bookkeeping function
        timers.insert(timer.name(), tempTimer);
    }

    return numErrors;
}

int Graph::setupCallbacks()
{
    int numErrors;
    for (const auto &callback : rosGraph.callbacks())
    {
        std::vector<std::string> validTopics;
        Graph::Callback tempCallback;
        tempCallback.execTimeUs = callback.exec_time_us();

        for (const auto &topic : callback.publishes_to())
        {
            if (topics.find(topic) == topics.end())
            {
                std::cerr << "Error: callback \"" << callback.name() << "\" publishes to non-existant topic \"" << topic << "\"\n";
                numErrors++;
                continue;
            }

            tempCallback.publishesTo.push_back(topic);
        }

        callbacks.insert(callback.name(), tempCallback);
    }

    return numErrors;
}

int Graph::setupSubscriptions()
{
    int numErrors = 0;
    for (const auto &subscription : rosGraph.subscriptions())
    {
        bool error = false;
        if (topics.find(subscription.topic()) == topics.end())
        {
            std::cerr << "Error: subscription ("
                      << subscription.topic() << ", " << subscription.callback() << ")"
                      << " references non-existant topic \"" << subscription.topic() << "\"\n";
            numErrors++;
            error = true;
        }

        if (callbacks.find(subscription.callback()) == callbacks.end())
        {
            std::cerr << "Error: subscription ("
                      << subscription.topic() << ", " << subscription.callback() << ")"
                      << " references non-existant callback \"" << subscription.callback() << "\"\n";
            numErrors++;
            error = true;
        }

        // If the subscription was valid, then set it up in ROS
        if (!error)
        {
            const std::string topic = subscription.topic();
            const std::string callbackName = subscription.callback();
            const uint32_t execTimeUs = callbacks.at(callbackName).execTimeUs;
            const std::vector<std::string> pubTopicList = callbacks.at(callbackName).publishesTo;

            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tempSub =
                this->create_subscription<std_msgs::msg::String>(
                    topic, 10, std::bind(&Graph::execCallback, this, callbackName, callbacks.at(callbackName)));

            subscriptions.push_back(tempSub);
        }
    }

    return numErrors;
}

void Graph::execCallback(const std::string name, const Graph::Callback cb)
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

void Graph::printGraphStats() const
{
    std::cout << "Graph Name: " << rosGraph.name() << "\n";

    std::cout << "Topics:\n";
    uint32_t count = 1;
    for (const auto &topic : rosGraph.topics())
    {
        std::cout << "\t" << count << ") " << topic.name() << "\n";
        count++;
    }

    count = 1;
    std::cout << "Timers: \n";
    for (const auto &timer : rosGraph.timers())
    {
        std::cout << "\t" << count << ") " << timer.name() << "\n";
        std::cout << "\t\tFrequency (Hz): " << timer.frequency_hz() << "\n";
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
    for (const auto &callback : rosGraph.callbacks())
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
    for (const auto &sub : rosGraph.subscriptions())
    {
        std::cout << "\t" << count << ") " << sub.topic()
                  << " --- " << sub.latency_us() << " us ---> " << sub.callback() << "\n";
    }
}