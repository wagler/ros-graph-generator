#include <iostream>
#include <string>
#include <Graph.hpp>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_graph_config_file> {single/multi}" << std::endl;
        return 1;
    }

    // Initialize the protobuf library
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // Get the file path from command line arguments
    std::string filePath = argv[1];

    // Start the ROS graph
    rclcpp::init(argc, argv);

	// True if single threaded executor is chosen (default)
	// False if multithreaded was chosen
	bool singleThreadedExecutor = true;
	singleThreadedExecutor = strcmp("multi",argv[2]);
	int numThreads = 1;

	if (!singleThreadedExecutor)
	{
		if (argc > 2 && argc != 4)
		{
			std::cerr << "Usage: " << argv[0] << " <path_to_graph_config_file> {single/multi} [num_threads]" << std::endl;
			std::cerr << "If you set the executor to `multi`, you must also provide the number of executor threads to use." << std::endl;
			return 1;
		}
		numThreads = atoi(argv[3]);
	}

    Graph graph(filePath, singleThreadedExecutor, numThreads);
    rclcpp::shutdown();

    // Cleanup protobuf
    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
