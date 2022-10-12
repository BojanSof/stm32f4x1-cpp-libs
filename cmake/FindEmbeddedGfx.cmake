include(FetchContent)

set(EMBEDDED_GFX_BUILD_EXAMPLES OFF)

FetchContent_Declare(
  EmbeddedGfx
  GIT_REPOSITORY "https://github.com/nikodinovska/EmbeddedGfx"
  GIT_TAG "v0.1.0"
)
FetchContent_MakeAvailable(EmbeddedGfx)