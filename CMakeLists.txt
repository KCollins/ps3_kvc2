cmake_minimum_required(VERSION 2.8.3)
project(ps3_kvc2)

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Executables
cs_add_executable(sine_client src/sine_client)
cs_add_executable(sine_server src/sine_server.cpp)
cs_add_executable(controller src/controller.cpp)
cs_add_executable(simulator src/simulator.cpp)

cs_install()
cs_export()
    
