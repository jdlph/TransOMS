cmake_minimum_required(VERSION 3.1.0)
project(TransOMS VERSION 0.1.0)

include(CTest)
enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

add_executable(${PROJECT_NAME}.gradient src/main.cpp src/elements.cpp src/ue.cpp src/utils.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

include_directories(include)

find_library(YAML-CPP yaml-cpp NAMES yaml-cpp_mac-x86 yaml-cpp_mac-arm yaml-cpp_linux PATHS lib)
target_link_libraries(${PROJECT_NAME}.gradient LINK_PUBLIC ${YAML-CPP})

# it is for std::experimental::filesystem in utils.cpp if g++ happens to be the compiler
if (CMAKE_COMPILER_IS_GNUCXX AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    target_link_libraries(${PROJECT_NAME}.gradient LINK_PUBLIC stdc++fs)
endif()