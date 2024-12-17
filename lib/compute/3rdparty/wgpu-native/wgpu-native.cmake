include(FetchContent)

if (NOT ARCH)
	set(SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR})
	if (SYSTEM_PROCESSOR STREQUAL "AMD64" OR SYSTEM_PROCESSOR STREQUAL "x86_64")
		if (CMAKE_SIZEOF_VOID_P EQUAL 8)
			set(ARCH "x86_64")
		elseif (CMAKE_SIZEOF_VOID_P EQUAL 4)
			set(ARCH "i686")
		endif()
	elseif (SYSTEM_PROCESSOR STREQUAL "arm64")
		set(ARCH "aarch64")
	endif()
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
	set(OS "windows")

	if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
		if (CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
			set(COMPILER "msvc")
		elseif (CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "GNU")
			set(COMPILER "gnu")
		endif()
	elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set(COMPILER "gnu")
	elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
		set(COMPILER "msvc")
	endif()

elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	set(OS "linux")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
	set(OS "macos")
else()
	message(FATAL_ERROR "Platform not supported by this release of UFO.")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
	set(WGPU_NATIVE_LIB_TYPE "release")
else()
	set(WGPU_NATIVE_LIB_TYPE "debug")
endif()

set(patch_command 
	${CMAKE_COMMAND} -E copy_if_different 
		"${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/wgpu-native/CMakeLists.txt.patch" 
		"<SOURCE_DIR>/CMakeLists.txt")

if(WIN32)
	FetchContent_Declare(
		wgpu-native
		URL      https://github.com/gfx-rs/wgpu-native/releases/download/v22.1.0.5/wgpu-${OS}-${ARCH}-${COMPILER}-${WGPU_NATIVE_LIB_TYPE}.zip
		PATCH_COMMAND ${patch_command}
	)
else()
	FetchContent_Declare(
		wgpu-native
		URL      https://github.com/gfx-rs/wgpu-native/releases/download/v22.1.0.5/wgpu-${OS}-${ARCH}-${WGPU_NATIVE_LIB_TYPE}.zip
		PATCH_COMMAND ${patch_command}
	)
endif()

FetchContent_MakeAvailable(wgpu-native)
