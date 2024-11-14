#ifndef UFO_PLAN_EDGE_HPP
#define UFO_PLAN_EDGE_HPP

// UFO
#include <ufo/math/vec.hpp>

// STL
#include <cstddef>
#include <memory>

namespace ufo
{
// Forward declare
template <std::size_t, class>
struct PlanNode;

template <std::size_t Dim, class T>
struct PlanEdge {
	std::shared_ptr<PlanNode<Dim, T>> from;
	std::shared_ptr<PlanNode<Dim, T>> to;
	T                                 cost{};
	T                                 distance{};

	PlanEdge() = default;

	PlanEdge(std::shared_ptr<PlanNode<Dim, T>> const& from,
	         std::shared_ptr<PlanNode<Dim, T>> const& to, T const& cost)
	    : from(from), to(to), cost(cost), distance(ufo::distance(*from, *to))
	{
	}

	PlanEdge(std::shared_ptr<PlanNode<Dim, T>> const& from,
	         std::shared_ptr<PlanNode<Dim, T>> const& to, T const& cost, T const& distance)
	    : from(from), to(to), cost(cost), distance(distance)
	{
	}
};
}  // namespace ufo

#endif  // UFO_PLAN_EDGE_HPP