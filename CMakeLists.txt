cmake_minimum_required(VERSION 3.4)

project(PandaKin
  VERSION 0.1
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3)
message("Eigen3 version: ${Eigen3_VERSION}")

# unfortunately not set by Eigen3
set(Eigen3_LIBRARIES Eigen3::Eigen)

include_directories(${EIGEN3_INCLUDE_DIRS})

set(PK_LIBS ${Eigen3_LIBRARIES})

set(PK_SOURCES kinematics.cpp panda_kinematics.cpp)

add_library(PandaKin SHARED ${PK_SOURCES})
target_link_libraries(PandaKin ${PK_LIBS})

add_executable(inv_panda inv_panda.cpp)
target_link_libraries(inv_panda PandaKin ${PK_LIBS})

add_executable(fwd_panda ${PK_SOURCES} fwd_panda.cpp)
target_link_libraries(fwd_panda ${PK_LIBS})
