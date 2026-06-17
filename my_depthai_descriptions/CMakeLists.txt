cmake_minimum_required(VERSION 3.8)
project(my_depthai_descriptions VERSION 3.3.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.py")
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})


ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/depthai_descriptions.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/depthai_descriptions.dsv.in")

ament_package()
