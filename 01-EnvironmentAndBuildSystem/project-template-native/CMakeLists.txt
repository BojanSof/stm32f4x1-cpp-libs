cmake_minimum_required (VERSION 3.18) 

project (
	project-template-native
	VERSION 0.1 
	DESCRIPTION "Project description"
	LANGUAGES C CXX ASM)

set(CMAKE_CONFIG_DIR "${CMAKE_SOURCE_DIR}/CMake")

add_subdirectory(app)
add_subdirectory(lib)
#add_subdirectory(tests) //@todo
