include("${PROJECT_SOURCE_DIR}/3rdparty/catch2/catch2.cmake")

# SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
# SET(GCC_COVERAGE_LINK_FLAGS    "--coverage")
# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
# SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )
# # set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

add_executable(ufoplan_tests
	plan_test.cpp
)

target_link_libraries(ufoplan_tests PRIVATE UFO::Container UFO::Map UFO::Math Catch2::Catch2WithMain)

target_compile_options(ufoplan_tests
	PUBLIC
		-Wall
		-Werror
		-Wextra
		-pedantic
		-Wconversion
		-Wcast-align
		-Wunused
		# -Wshadow
		-Wold-style-cast
		-Wpointer-arith
		-Wcast-qual
		-Wno-missing-braces
)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(ufoplan_tests)

# add_custom_command(
# 	TARGET ufoplan_tests
# 	COMMENT "Running UFO plan tests"
# 	POST_BUILD 
# 	COMMAND ufoplan_tests
# )