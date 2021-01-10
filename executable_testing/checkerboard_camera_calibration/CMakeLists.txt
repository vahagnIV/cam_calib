find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(calibrate main.cpp)

target_link_libraries(calibrate
        ${OpenCV_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        g2o_learning)

