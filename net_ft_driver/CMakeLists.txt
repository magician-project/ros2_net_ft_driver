cmake_minimum_required(VERSION 3.8)
project(net_ft_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ASIO
  CURLPP
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  TinyXML2
  std_msgs
  std_srvs
  geometry_msgs
  diagnostic_msgs
  diagnostic_updater
)

find_package(ament_cmake REQUIRED)
find_package(asio_cmake_module REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  if(${Dependency} STREQUAL "CURLPP")
    include(FindPkgConfig)
    pkg_check_modules(${Dependency} REQUIRED curlpp)
  else()
    find_package(${Dependency} REQUIRED)
  endif()
endforeach()

include_directories(
  include
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/hardware_interface.cpp
  src/interfaces/net_ft_interface.cpp
  src/interfaces/ati_ft_interface.cpp
  src/interfaces/ati_axia_ft_interface.cpp
  src/interfaces/onrobot_ft_interface.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "srv/SetInt.srv"
  LIBRARY_NAME ${PROJECT_NAME}
   #DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(
  cpp_typesupport_target 
  ${PROJECT_NAME}_interfaces
  "rosidl_typesupport_cpp"
)

add_library( ${PROJECT_NAME}_ati_ft_sensor
  SHARED
  src/ATIFTSensor.cpp
)
ament_target_dependencies( ${PROJECT_NAME}_ati_ft_sensor
  rclcpp
  std_srvs
  geometry_msgs
)
target_link_libraries( ${PROJECT_NAME}_ati_ft_sensor
  ${PROJECT_NAME}
  "${cpp_typesupport_target}"
)

add_executable(
  ati_ft_main
  src/ati_ft_main.cpp
)
ament_target_dependencies(
  ati_ft_main
  rclcpp
)
target_link_libraries(
  ati_ft_main
  ${PROJECT_NAME}_ati_ft_sensor
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  TARGETS ${PROJECT_NAME}_ati_ft_sensor
  DESTINATION lib
)

install(
  TARGETS ati_ft_main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
