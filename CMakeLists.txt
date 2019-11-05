cmake_minimum_required(VERSION 2.8.3)
project(hebi_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package(
)

include_directories(
  .
  ${catkin_INCLUDE_DIRS}
)
add_custom_target(${PROJECT_NAME}_generate_xacro ALL 
	${PROJECT_SOURCE_DIR}/scripts/generate_pipeline_batch.bash
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

## Mark files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  meshes
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

