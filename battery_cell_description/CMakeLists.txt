cmake_minimum_required(VERSION 3.8)
project(battery_cell_description)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY
        config 
        launch 
        urdf
        meshes
    DESTINATION
        share/${PROJECT_NAME}
)

install(PROGRAMS
        scripts/clock_node.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()