// UFO
#include <ufo/map/semantic/map.hpp>
#include <ufo/map/ufomap.hpp>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

TEST_CASE("SemanticMap")
{
	SECTION("Semantic set")
	{
		Map3D<SemanticMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2);

		Semantic s(5, 0.2);
		map.semanticSet(coord, s);

		auto ret = map.semantic(coord);
		REQUIRE(ret == s);
	}
}
