/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_MAP_SEMANTIC_MAP_HPP
#define UFO_MAP_SEMANTIC_MAP_HPP

// UFO
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/map/block.hpp>
#include <ufo/map/semantic/block.hpp>
// #include <ufo/map/semantic/predicate.hpp>
#include <ufo/map/type.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <iostream>
#include <type_traits>

namespace ufo
{
template <class Derived, class Tree>
class SemanticMap
{
 private:
	template <class Derived2, class Tree2>
	friend class SemanticMap;

	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::SEMANTIC;

	// Container
	using Index    = typename Tree::Index;
	using Node     = typename Tree::Node;
	using Code     = typename Tree::Code;
	using Key      = typename Tree::Key;
	using Point    = typename Tree::Point;
	using Coord    = typename Tree::Coord;
	using coord_t  = typename Tree::coord_t;
	using depth_t  = typename Tree::depth_t;
	using offset_t = typename Tree::offset_t;
	using length_t = typename Tree::length_t;
	using pos_t    = typename Tree::pos_t;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Semantic semantic(NodeType node) const
	{
		Index n = derived().index(node);
		return semanticBlock(n.pos)[n.offset].semantic;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Modifiers                                        |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief
	 *
	 * @note If `NodeType` is `Index`, only propagates up to `node` and it does not set
	 * modified if propagation is `false`.
	 *
	 * @tparam NodeType Should be of type `Node`, `Code`, `Key`, or `Coord`
	 * @param node
	 * @param value
	 * @param propagate
	 */
	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void semanticSet(NodeType node, Semantic value, bool propagate = true)
	{
		SemanticElement elem(value);

		auto node_f  = [this, elem](Index node) { semanticBlock(node.pos)[node.offset] = elem; };
		auto block_f = [this, elem](pos_t pos) { semanticBlock(pos).fill(elem); };

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>        = true,
	          std::enable_if_t<std::is_invocable_r_v<Semantic, UnaryOp, Index>, bool> = true>
	void semanticUpdate(NodeType node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			Semantic value = unary_op(node);

			auto& sb = SemanticBlock(node.pos)[node.offset];
			sb.semantic = value;
		};

		auto block_f = [this, node_f](pos_t pos) {
			for (std::size_t i{}; BF > i; ++i) {
				node_f(Index(pos, i));
			}
		};

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	SemanticMap() { onInitRoot(); }

	SemanticMap(SemanticMap const& other) = default;

	SemanticMap(SemanticMap&& other) = default;

	template <class Derived2, class Tree2>
	SemanticMap(SemanticMap<Derived2, Tree2> const& other)
	{
	}

	template <class Derived2, class Tree2>
	SemanticMap(SemanticMap<Derived2, Tree2>&& other)
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~SemanticMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	SemanticMap& operator=(SemanticMap const& rhs) = default;

	SemanticMap& operator=(SemanticMap&& rhs) = default;

	template <class Derived2, class Tree2>
	SemanticMap& operator=(SemanticMap<Derived2, Tree2> const& rhs)
	{
		return *this;
	}

	template <class Derived2, class Tree2>
	SemanticMap& operator=(SemanticMap<Derived2, Tree2>&& rhs)
	{
		return *this;
	}

	//
	// Swap
	//

	void swap(SemanticMap& other) noexcept {}

	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Block                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] SemanticBlock<BF>& semanticBlock(pos_t pos)
	{
		return derived().template data<SemanticBlock<BF>>(pos);
	}

	[[nodiscard]] SemanticBlock<BF> const& semanticBlock(pos_t pos) const
	{
		return derived().template data<SemanticBlock<BF>>(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expect                               |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot()
	{
		auto& block = semanticBlock(0);

		Semantic value{};
		block[0].semantic = value;
	}

	void onInitChildren(Index node, pos_t children)
	{
		semanticBlock(children).fill(semanticBlock(node.pos)[node.offset]);
	}

	void onPropagateChildren(Index node, pos_t children)
	{
		auto const& children_block = semanticBlock(children);
		auto&       sb             = semanticBlock(node.pos)[node.offset];

		Semantic value{};
		for (std::size_t i{}; BF > i; ++i) {
			value.label |= children_block[i].semantic.label;
			// TODO: Propagate value criteria? Highest, lowest, average, none?
			value.value = std::max(value.value, children_block[i].semantic.value);
		}

		sb.semantic = value;
	}

	[[nodiscard]] bool onIsPrunable(pos_t block) const
	{
		return std::all_of(
		    begin(semanticBlock(block).data) + 1, end(semanticBlock(block).data),
		    [v = semanticBlock(block)[0]](auto const& e) { return v == e; });
	}

	void onPruneChildren(Index node, pos_t children) {}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] constexpr std::size_t serializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */,
	    std::size_t num_nodes) const
	{
		return num_nodes * sizeof(Semantic);
	}

	void onRead(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			auto& sb = semanticBlock(block);

			if (offset.all()) {
				std::array<Semantic, BF> semantic;
				in.read(semantic);
				for (offset_t i{}; BF > i; ++i) {
					sb[i] = SemanticElement(semantic[i]);
				}
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						Semantic value;
						in.read(value);
						sb[i] = SemanticElement(value);
					}
				}
			}
		}
	}

	void onWrite(WriteBuffer&                                     out,
	             std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes) const
	{
		for (auto [block, offset] : nodes) {
			auto const& sb = SemanticBlock(block);

			if (offset.all()) {
				std::array<Semantic, BF> semantic;
				for (offset_t i{}; BF > i; ++i) {
					semantic[i] = sb[i].semantic;
				}
				out.write(semantic);
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						out.write(sb[i].semantic);
					}
				}
			}
		}
	}

	//
	// Dot file info
	//

	void onDotFileInfo(std::ostream& out, Index node) const
	{
		out << "Semantic: " << semantic(node);
	}
};

template <std::size_t Dim, std::size_t BF>
struct map_block<SemanticMap, Dim, BF> {
	using type = SemanticBlock<BF>;
};
}  // namespace ufo

#endif