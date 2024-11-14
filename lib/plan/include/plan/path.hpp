#ifndef UFO_PLAN_PATH_HPP
#define UFO_PLAN_PATH_HPP

// UFO
#include <ufo/math/vec.hpp>
#include <ufo/plan/node.hpp>

// STL
#include <cstddef>
#include <vector>

namespace ufo
{
template <std::size_t Dim, class T>
using PlanPath = std::vector<std::shared_ptr<PlanNode<Dim, T>>>;
}  // namespace ufo

#endif  // UFO_PLAN_PATH_HPP