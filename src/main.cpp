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

    // Get the file path from command line arguments
    std::string filePath = argv[1];
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Graph>(filePath));
    rclcpp::shutdown();

    return 0;
}
