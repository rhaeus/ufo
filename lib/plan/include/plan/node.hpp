#ifndef UFO_PLAN_NODE_HPP
#define UFO_PLAN_NODE_HPP

// UFO
#include <ufo/math/vec.hpp>
#include <ufo/plan/edge.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

namespace ufo
{
template <std::size_t Dim, class T>
struct PlanNode
    : public std::enable_shared_from_this<PlanNode<Dim, T>>
    , public Vec<Dim, T> {
	std::vector<PlanEdge<Dim, T>> edges;
	T                             cost;

	PlanNode() = default;

	PlanNode(Vec<Dim, T> const& position, T const& cost) : Vec<Dim, T>(position), cost(cost)
	{
	}

	bool addEdge(std::shared_ptr<PlanNode> const& neighbor, T const& cost)
	{
		if (nullptr == neighbor) {
			return false;
		}

		for (auto const& edge : edges) {
			if (edge.to == neighbor) {
				return false;
			}
		}

		edges.emplace_back(std::enable_shared_from_this<PlanNode<Dim, T>>::shared_from_this(),
		                   neighbor, cost);
		return true;
	}

	bool addEdge(std::shared_ptr<PlanNode> const& neighbor, T const& cost,
	             T const& distance)
	{
		if (nullptr == neighbor) {
			return false;
		}

		for (auto const& edge : edges) {
			if (edge.to == neighbor) {
				return false;
			}
		}

		edges.emplace_back(std::enable_shared_from_this<PlanNode<Dim, T>>::shared_from_this(),
		                   neighbor, cost, distance);
		return true;
	}

	bool eraseEdge(std::shared_ptr<PlanNode> const& neighbor)
	{
		auto it = std::remove_if(edges.begin(), edges.end(), [&neighbor](auto const& edge) {
			return edge.to == neighbor;
		});
		auto erased = edges.end() != it;
		auto r      = edges.end() - it;
		edges.erase(it, edges.end());
		return erased;
	}

 protected:
	// For A* planning

	bool      visited_   = false;
	PlanNode* came_from_ = nullptr;
	T         g_score_   = std::numeric_limits<T>::max();

	void resetPlanningState()
	{
		visited_   = false;
		came_from_ = nullptr;
		g_score_   = std::numeric_limits<T>::max();
	}

	template <std::size_t, class>
	friend class PlanGraph;
};
}  // namespace ufo

#endif  // UFO_PLAN_NODE_HPP