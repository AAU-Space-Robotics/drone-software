cmake_minimum_required(VERSION 3.10)
project(gcs)

# Use C++14 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Fix OpenGL GLVND issue by setting the policy
if (POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

# Fetch Dear ImGui, GLFW, ImPlot, and tinyxml2 using FetchContent
include(FetchContent)

# ImGui
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(imgui)

# GLFW
FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG        3.3.4
)
FetchContent_MakeAvailable(glfw)

# ImPlot
FetchContent_Declare(
  implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(implot)

# tinyxml2
FetchContent_Declare(
  tinyxml2
  GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(tinyxml2)


# Find OpenGL package
find_package(OpenGL REQUIRED)

# Include directories for external libraries
include_directories(
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
  ${glfw_SOURCE_DIR}/include
  ${implot_SOURCE_DIR}      # Add ImPlot include directory
  ${tinyxml2_SOURCE_DIR}    # Add tinyxml2 include directory
  ${CMAKE_SOURCE_DIR}/include
)

# Add executable for the GUI node
add_executable(gcs_node
  src/gcs.cpp
  src/gcslib.cpp
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  ${implot_SOURCE_DIR}/implot.cpp           # Add ImPlot source
  ${implot_SOURCE_DIR}/implot_items.cpp     # Add ImPlot items source
)

# Manual controller node executable
install(PROGRAMS
  script/manual_controller.py
  script/manual_arrow_controller.py
  DESTINATION lib/${PROJECT_NAME}/scripts
)

# Link the necessary libraries
target_link_libraries(gcs_node
  OpenGL::GL      # Link OpenGL
  glfw            # Link GLFW
  tinyxml2        # Link tinyxml2 from FetchContent
)

# Ensure ROS 2 dependencies are added correctly
ament_target_dependencies(gcs_node
  rclcpp
  std_msgs
  interfaces
)

# Install the executable
install(TARGETS gcs_node
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()
