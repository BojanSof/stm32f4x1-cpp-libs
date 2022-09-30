cmake_minimum_required(VERSION 3.18)

project(lib2)

add_library(lib2 src/isqrt.cpp)
target_compile_features(lib2 PRIVATE cxx_std_17)

target_include_directories(lib2
	PUBLIC
		include
	PRIVATE
		src
) 
