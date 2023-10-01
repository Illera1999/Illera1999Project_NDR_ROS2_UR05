cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#Ejecutables
# find dependencies
add_library(neuron STATIC IMPORTED)
set_target_properties(neuron PROPERTIES IMPORTED_LOCATION C:/dev/project/capture_motion_ros2/src/my_cpp_pkg/neuron_reader/lib/NeuronDataReader.lib)

#add_library(sdl C:/dev/project/capture_motion_ros2/src/my_cpp_pkg/neuron_reader/lib/NeuronDataReader.dll)
#set_target_properties(sdl PROPERTIES LINKER_LANGUAGE CXX)

add_executable(cpp_node src/readExport.cpp)
ament_target_dependencies(cpp_node rclcpp)

target_include_directories(cpp_node PUBLIC neuron_reader/include)
target_link_libraries(cpp_node neuron)
#target_link_libraries(cpp_node sdl)


install(TARGETS
  cpp_node DESTINATION lib/${PROJECT_NAME}
)

ament_package()
