cmake_minimum_required(VERSION 3.18)

project(app)

add_executable(app src/main.cpp)
target_compile_features(app PRIVATE cxx_std_17)
target_link_libraries(app PRIVATE lib1)
target_link_libraries(app PRIVATE lib2)

target_include_directories(app
	PUBLIC
		include
	PRIVATE
		src
) 
 
