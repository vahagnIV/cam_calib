

add_library(g2o_learning
        intrinsic_solver.cpp
        intrinsic_solver.h
        vertex_camera.h
        vertex_camera.cpp
        edge_error.h)

target_link_libraries(g2o_learning  g2o::core g2o::solver_cholmod g2o::types_sba g2o::opengl_helper Eigen3::Eigen)

target_include_directories(g2o_learning INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})