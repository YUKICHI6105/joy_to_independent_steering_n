#include <joy_to_independent_steering_n/node.hpp>

int main(int argc, char * argv[])
{
	std::cout << "node started." << std::endl;
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exec{};
	rclcpp::NodeOptions options{};

	std::cout << "initilalizing..." << std::endl;

	auto joy_to_indster = std::make_shared<nhk2024::joy_to_independent_steering_n::node::Node>("joy_to_independent_steering_n", "independent_steering_n", options);
	exec.add_node(joy_to_indster);
	
	std::cout << "start spinning." << std::endl;

	exec.spin();
	
	std::cout << "node ended." << std::endl;

	rclcpp::shutdown();
}
