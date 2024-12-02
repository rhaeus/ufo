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

#ifndef UFO_GEOMETRY_BOX_HPP
#define UFO_GEOMETRY_BOX_HPP

// UFO
#include <ufo/math/vec.hpp>

// STL
#include <cstddef>
#include <ostream>
#include <type_traits>

namespace ufo
{
/*!
 * @brief Something something
 * @author Daniel Duberg
 *
 */
template <std::size_t Dim = 3, class T = float>
struct Box {
	static_assert(std::is_floating_point_v<T>, "T is required to be floating point.");

	using value_type = T;

	Vec<Dim, T> min;
	Vec<Dim, T> max;

	constexpr Box() noexcept = default;

	constexpr Box(Vec<Dim, T> const& min, Vec<Dim, T> const& max) noexcept
	    : min(min), max(max)
	{
	}

	constexpr Box(Vec<Dim, T> const& center, T half_length) noexcept
	    : min(center - half_length), max(center + half_length)
	{
	}

	constexpr Box(Box const&) noexcept = default;

	template <class U>
	constexpr explicit Box(Box<Dim, U> const& other) noexcept
	    : min(other.min), max(other.max)
	{
	}

	[[nodiscard]] static constexpr std::size_t size() noexcept { return Dim; }

	[[nodiscard]] constexpr Vec<Dim, T> center() const noexcept
	{
		return min + halfLength();
	}

	[[nodiscard]] constexpr Vec<Dim, T> length() const noexcept { return max - min; }

	[[nodiscard]] constexpr Vec<Dim, T> halfLength() const noexcept
	{
		return length() * T(0.5);
	}
};

//
// Deduction guide
//

template <std::size_t Dim, class T>
Box(Vec<Dim, T>, Vec<Dim, T>) -> Box<Dim, T>;

template <std::size_t Dim, class T>
Box(Vec<Dim, T>, T) -> Box<Dim, T>;

/*!
 * @brief Compare two Boxs.
 *
 * @param lhs,rhs The Boxs to compare
 * @return `true` if they compare equal, `false` otherwise.
 */
template <std::size_t Dim, class T>
bool operator==(Box<Dim, T> const& lhs, Box<Dim, T> const& rhs)
{
	return lhs.min == rhs.min && lhs.min == rhs.min;
}

/*!
 * @brief Compare two Boxs.
 *
 * @param lhs,rhs The Boxs to compare
 * @return `true` if they do not compare equal, `false` otherwise.
 */
template <std::size_t Dim, class T>
bool operator!=(Box<Dim, T> const& lhs, Box<Dim, T> const& rhs)
{
	return !(lhs == rhs);
}

template <std::size_t Dim, class T>
std::ostream& operator<<(std::ostream& out, Box<Dim, T> const& box)
{
	return out << "Min: " << box.min << ", Max: " << box.max;
}

template <class T>
using Box1 = Box<1, T>;
template <class T>
using Box2 = Box<2, T>;
template <class T>
using Box3 = Box<3, T>;
template <class T>
using Box4 = Box<4, T>;

using Box1f = Box<1, float>;
using Box2f = Box<2, float>;
using Box3f = Box<3, float>;
using Box4f = Box<4, float>;

using Box1d = Box<1, double>;
using Box2d = Box<2, double>;
using Box3d = Box<3, double>;
using Box4d = Box<4, double>;
}  // namespace ufo

#endif  // UFO_GEOMETRY_BOX_HPP