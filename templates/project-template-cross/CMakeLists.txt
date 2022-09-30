cmake_minimum_required(VERSION 3.18)

project(
  stm32-cmake-template
  VERSION 0.1
  DESCRIPTION "Template for stm32 embedded programming"
  LANGUAGES C CXX ASM)

list(APPEND CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_LIST_DIR}/cmake")

add_subdirectory(src)
