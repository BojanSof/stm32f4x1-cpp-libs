include(FetchContent)

set(FSM_GENERATOR_BUILD_EXAMPLES OFF)

FetchContent_Declare(
  FsmGenerator
  GIT_REPOSITORY "https://github.com/BojanSof/FSM-Generator"
  GIT_TAG "v0.1.0"
)
FetchContent_MakeAvailable(FsmGenerator)