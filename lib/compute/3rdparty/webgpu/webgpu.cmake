include(FetchContent)

FetchContent_Declare(
	webgpu
	URL	https://github.com/gfx-rs/wgpu-native/releases/download/v0.19.3.1/wgpu-linux-x86_64-release.zip
	# URL_HASH SHA512=916b6a8409c6f32d9d939c7cc1581795a1c019efc2bf21d701cc69ed4b9359ab0123eeb872a23061087fd82d8d367580d92ce9b73440db089ea58d39776fdaf2
	PATCH_COMMAND       patch -p2 < ${CMAKE_CURRENT_SOURCE_DIR}/cmake/webgpu.patch
	UPDATE_DISCONNECTED 1
)

FetchContent_MakeAvailable(webgpu)
