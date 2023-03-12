cmake_minimum_required(VERSION 3.0.0)
project(TransOMS VERSION 0.1.0)

include(CTest)
enable_testing()

set(CMAKE_CXX_STANDARD 17)
add_executable(${PROJECT_NAME}.gradient src/main.cpp src/elements.cpp src/ue.cpp src/utils.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

include_directories(include)
include_directories(include/yaml-cpp)

find_library(YAML-CPP yaml-cpp yaml-cpp_linux lib)
target_link_libraries(${PROJECT_NAME}.gradient LINK_PUBLIC ${YAML-CPP})

# it is for std::experimental::filesystem from g++ in utils.cpp
if (CMAKE_COMPILER_IS_GNUCXX AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    target_link_libraries(${PROJECT_NAME}.gradient LINK_PUBLIC stdc++fs)
endif()