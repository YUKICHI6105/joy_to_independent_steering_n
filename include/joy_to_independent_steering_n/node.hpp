#pragma once

#include <chrono>
#include <string>
#include <string_view>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int8.hpp>
#include <independent_steering_n/msg/linear_velocity.hpp>
#include <independent_steering_n/msg/angular_velocity.hpp>
#include <independent_steering_n/control_mode.hpp>

#include "input/joy_to_key_button.hpp"
#include "input/logicool.hpp"
#include "utility.hpp"

namespace nhk2024::joy_to_independent_steering_n::node
{
	constexpr float epsilon = 0.1f;
	constexpr float deadzone = 0.1f;

	class Node : public rclcpp::Node
	{
		rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr control_mode_pub;
		rclcpp::Publisher<::independent_steering_n::msg::LinearVelocity>::SharedPtr linear_velocity_pub;
		rclcpp::Publisher<::independent_steering_n::msg::AngularVelocity>::SharedPtr angular_velocity_pub;

		CRSLib::Ros2::Logicool logicool;
		independent_steering_n::control_mode::ControlMode mode{independent_steering_n::control_mode::ControlMode::disable};
		float last_vx{0.0f};
		float last_vy{0.0f};

		rclcpp::TimerBase::SharedPtr timer{};

		public:
		Node(const std::string_view node_name, const std::string_view steering_node_name, const rclcpp::NodeOptions& options) noexcept:
			rclcpp::Node(std::string(node_name), options),
			control_mode_pub{create_publisher<std_msgs::msg::UInt8>(std::string(steering_node_name) + "/control_mode", 10)},
			linear_velocity_pub{create_publisher<::independent_steering_n::msg::LinearVelocity>(std::string(steering_node_name) + "/linear_velocity", 1)},
			angular_velocity_pub{create_publisher<::independent_steering_n::msg::AngularVelocity>(std::string(steering_node_name) + "/angular_velocity", 1)},
			logicool{*this, "joy"}
		{
			using namespace std::chrono_literals;
			timer = this->create_wall_timer(10ms, std::bind(&Node::callback, this));
		}

		private:
		void callback()
		{
			using CRSLib::Ros2::Logicool;
			using nhk2024::independent_steering_n::control_mode::ControlMode;

			if(logicool.is_pushed_down(Logicool::Buttons::a))
			{
				std_msgs::msg::UInt8 msg{};
				msg.data = utility::to_underlying(ControlMode::crab);
				control_mode_pub->publish(msg);
				mode = ControlMode::crab;
			}
			else if(logicool.is_pushed_down(Logicool::Buttons::b))
			{
				std_msgs::msg::UInt8 msg{};
				msg.data = utility::to_underlying(ControlMode::spinning);
				control_mode_pub->publish(msg);
				mode = ControlMode::spinning;
			}
			else if(logicool.is_pushed_down(Logicool::Buttons::x))
			{
				std_msgs::msg::UInt8 msg{};
				msg.data = utility::to_underlying(ControlMode::disable);
				control_mode_pub->publish(msg);
				mode = ControlMode::disable;
			}

			switch(mode)
			{
				case ControlMode::disable:
				break;

				case ControlMode::crab:
				{
					const auto axes = logicool.get_axes();
					const auto vx = -axes[Logicool::Axes::l_stick_LR];
					const auto vy = axes[Logicool::Axes::l_stick_UD];
					if(logicool.is_being_pushed(Logicool::Buttons::y) || (deadzone < vx * vx + vy * vy && last_vx * last_vx + last_vy * last_vy <= vx * vx + vy * vy + epsilon))
					{
						auto msg = ::independent_steering_n::msg::LinearVelocity{};
						msg.x = vx;
						msg.y = vy;
						linear_velocity_pub->publish(msg);
					}
					last_vx = vx;
					last_vy = vy;
				}
				break;

				case ControlMode::spinning:
				{
					const auto axes = logicool.get_axes();
					const auto yaw = axes[Logicool::Axes::r_stick_LR];
					auto msg = ::independent_steering_n::msg::AngularVelocity{};
					msg.yaw = yaw;
					angular_velocity_pub->publish(msg);
				}
				break;
			}
		}

	};
}