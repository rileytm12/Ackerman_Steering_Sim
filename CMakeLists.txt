cmake_minimum_required(VERSION 3.14)
project(ackerman_steering_sim CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 QUIET CONFIG)
if(NOT TARGET Eigen3::Eigen)
    include(FetchContent)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(eigen)
    if(NOT eigen_POPULATED)
        FetchContent_Populate(eigen)
    endif()
    add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
    set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}")
endif()

# main.cpp pulls in other .cpp files via #include
add_executable(ackerman main.cpp)
target_link_libraries(ackerman PRIVATE Eigen3::Eigen)
