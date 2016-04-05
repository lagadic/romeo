cmake_minimum_required(VERSION 2.8.3)
project(romeo_description_mc)

# We need to find catkin for the rest of the script to work
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  message_filters
  sensor_msgs
  xacro
)

# Nothing that can be imported by other modules
catkin_package(
 CATKIN_DEPENDS xacro mc_rbdyn_urdf
)

include_directories(${catkin_INCLUDE_DIRS})


# Generate URDF models.
set(urdf_files "")
macro(new_generate_urdf input)
	string(REPLACE ".xacro" ".urdf" "output" "${input}")
	message("processing " ${input} "into " ${output})
	xacro_add_xacro_file(${input} ${output})
	set(urdf_files ${urdf_files} ${output})
endmacro()

new_generate_urdf(${PROJECT_SOURCE_DIR}/urdf/romeo.xacro)
add_custom_target(romeo_media_files ALL DEPENDS ${urdf_files})

catkin_python_setup()



# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
# Instructions to install config files
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config )
# Instructions to install urdf files
install(DIRECTORY urdf/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
	PATTERN "*.xacro" EXCLUDE)

install(DIRECTORY convex/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/convex)

install(DIRECTORY stpbv/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/stpbv)

install(DIRECTORY meshes/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY rsdf/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rsdf)

install(DIRECTORY surfaces/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/surfaces)
