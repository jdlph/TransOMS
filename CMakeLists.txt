cmake_minimum_required(VERSION 3.1.0)
project(TransOMS VERSION 0.1.0)

include(CTest)
enable_testing()

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Release)

add_executable(${PROJECT_NAME} src/main.cpp src/elements.cpp src/ue.cpp src/utils.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

include_directories(include)

# default name for Windows
set(YAML_CPP_LIB_NAME yaml-cpp)

if (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64")
        set(YAML_CPP_LIB_NAME yaml-cpp_mac-arm)
    elseif(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(YAML_CPP_LIB_NAME yaml-cpp_mac-x86)
    endif()
    set(CMAKE_CXX_FLAGS "-Xpreprocessor -fopenmp-version=50 -fpic")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(YAML_CPP_LIB_NAME yaml-cpp_linux)
    set(CMAKE_CXX_FLAGS "-fopenmp -fpic")
endif()

find_package(OpenMP REQUIRED)
find_library(YAML_CPP ${YAML_CPP_LIB_NAME} PATHS lib)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${YAML_CPP})
target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)

# it is for std::experimental::filesystem in utils.cpp if g++ happens to be the compiler
if (CMAKE_COMPILER_IS_GNUCXX)
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC stdc++fs)
endif()