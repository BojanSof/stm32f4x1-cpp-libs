include(FetchContent)

set(EMBEDDED_GFX_BUILD_EXAMPLES OFF)

FetchContent_Declare(
  EmbeddedGfx
  GIT_REPOSITORY "https://github.com/nikodinovska/EmbeddedGfx"
  GIT_TAG "origin/main"
)
FetchContent_MakeAvailable(EmbeddedGfx)