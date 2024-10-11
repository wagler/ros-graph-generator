#include <iostream>
#include <string>
#include <Graph.hpp>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_graph_config_file>" << std::endl;
        return 1;
    }

    // Get the file path from command line arguments
    std::string filePath = argv[1];

    Graph rosGraph(filePath);

    return 0;
}
