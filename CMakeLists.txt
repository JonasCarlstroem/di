cmake_minimum_required(VERSION 3.23)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

CREATE_LIB(di INTERFACE)
#add_library(di INTERFACE)

target_compile_features(di INTERFACE cxx_std_17)

target_include_directories(di INTERFACE 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/di>
)

INSTALL_LIB(di True di)