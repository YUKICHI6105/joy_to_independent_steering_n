#pragma once

#include <type_traits>

namespace nhk2024::joy_to_independent_steering_n::utility
{
	template<class T>
	requires std::is_enum_v<T>
	inline auto to_underlying(const T value) noexcept
	{
		return static_cast<std::underlying_type_t<T>>(value);
	}
}