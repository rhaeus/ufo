Include(FetchContent)

FetchContent_Declare(
  ufomap
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufomap.git
  GIT_TAG        v2
  GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(ufomap)