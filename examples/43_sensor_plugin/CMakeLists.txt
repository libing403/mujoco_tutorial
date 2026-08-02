cmake_minimum_required(VERSION 3.16)
project(43_sensor_plugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(MUJOCO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../mujoco-3.11.0)

add_library(book_sensor_plugin SHARED main.cc)
target_compile_definitions(book_sensor_plugin PRIVATE BUILD_PLUGIN)
target_include_directories(book_sensor_plugin PRIVATE ${MUJOCO_ROOT}/include)
target_link_directories(book_sensor_plugin PRIVATE ${MUJOCO_ROOT}/lib)
target_link_libraries(book_sensor_plugin PRIVATE mujoco)
set_target_properties(book_sensor_plugin PROPERTIES BUILD_RPATH ${MUJOCO_ROOT}/lib)

add_executable(demo main.cc)
target_include_directories(demo PRIVATE ${MUJOCO_ROOT}/include)
target_link_directories(demo PRIVATE ${MUJOCO_ROOT}/lib)
target_link_libraries(demo PRIVATE mujoco)
set_target_properties(demo PROPERTIES BUILD_RPATH ${MUJOCO_ROOT}/lib)
