cmake_minimum_required(VERSION 3.5...3.16)

project(ufocompute
	VERSION 2.0.0
	DESCRIPTION "The UFO compute library"
	LANGUAGES CXX
)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(UFO_TESTING  "Unit testing build" OFF)
option(UFO_COVERAGE "Test coverage" OFF)
option(UFO_DOCS     "Documentation build" OFF)

# add_subdirectory(1stparty)

add_library(ufocompute INTERFACE)
add_library(UFO::Compute ALIAS ufocompute)

set_target_properties(ufocompute PROPERTIES
	INTERFACE_VERSION ${PROJECT_VERSION}
	INTERFACE_SOVERSION ${PROJECT_VERSION_MAJOR}
	INTERFACE_CXX_STANDARD 17
	INTERFACE_CXX_EXTENSIONS OFF
)

# target_link_libraries(ufocompute INTERFACE UFO::Utility)

target_include_directories(ufocompute INTERFACE
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_compile_features(ufocompute INTERFACE cxx_std_17)

include(GNUInstallDirs)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(UFO_TESTING)
  add_subdirectory(tests)
endif()

if(UFO_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS ufocompute
	EXPORT ufocomputeTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT ufocomputeTargets
	FILE "ufocomputeTargets.cmake"
	NAMESPACE UFO::
	DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
	"${PROJECT_SOURCE_DIR}/cmake/ufocomputeConfig.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/ufocomputeConfig.cmake"
	INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/ufocomputeConfigVersion.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

INSTALL(
	FILES
	"${CMAKE_CURRENT_BINARY_DIR}/ufocomputeConfig.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/ufocomputeConfigVersion.cmake"
	DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
	DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include
)