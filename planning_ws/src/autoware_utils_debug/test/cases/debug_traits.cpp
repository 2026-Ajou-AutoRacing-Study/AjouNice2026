#include <autoware_utils_debug/debug_traits.hpp>
static_assert(!autoware_utils_debug::debug_traits::is_debug_message<int>::value, "non-debug type");
