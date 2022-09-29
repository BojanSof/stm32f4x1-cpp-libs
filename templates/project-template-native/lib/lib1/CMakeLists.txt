cmake_minimum_required(VERSION 3.18)

project(lib1)

add_library(lib1 src/mysqrt.cpp)
target_compile_features(lib1 PRIVATE cxx_std_17)

target_include_directories(lib1
	PUBLIC
		include
	PRIVATE
		src
) 
