#include <iostream>
#include <string>
#include <Graph.hpp>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_graph_config_file>" << std::endl;
        return 1;
    }

    // Initialize the protobuf library
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // Get the file path from command line arguments
    std::string filePath = argv[1];

    // Start the ROS graph
    rclcpp::init(argc, argv);
    Graph graph(filePath);
    rclcpp::shutdown();

    // Cleanup protobuf
    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
