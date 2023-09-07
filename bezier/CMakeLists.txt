# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(bezier bezier_for_drive.cpp)
target_link_libraries(bezier ${Pangolin_LIBRARIES})
