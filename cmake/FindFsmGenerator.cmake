include(FetchContent)

set(FSM_GENERATOR_BUILD_EXAMPLES OFF)

FetchContent_Declare(
  FsmGenerator
  GIT_REPOSITORY "https://github.com/BojanSof/FSM-Generator"
  GIT_TAG "origin/main"
)
FetchContent_MakeAvailable(FsmGenerator)