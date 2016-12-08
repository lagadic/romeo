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
 SKIP_PKG_CONFIG_GENERATION
 CATKIN_DEPENDS xacro mc_rbdyn_urdf
)

include_directories(${catkin_INCLUDE_DIRS})

# Configure the develspace pkg-config file
set(DATADIR "${PROJECT_SOURCE_DIR}")
configure_file(
  "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}.pc.cmake"
  "${CATKIN_DEVEL_PREFIX}/lib/pkgconfig/${PROJECT_NAME}.pc")


# Configure the installspace pkg-config file
set(DATADIR "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}")
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace)
configure_file(
  "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}.pc.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}.pc")
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}.pc
  DESTINATION lib/pkgconfig)


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
