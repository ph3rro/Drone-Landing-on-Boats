# - Config to retrieve all components of the asv_sim2 package
#
# This should only be invoked by asv_sim2-config.cmake.
#
# To retrieve this meta-package, use:
# find_package(asv_sim2 COMPONENTS all)
#
# This creates the target asv_sim2::all which will link to all known
# components of asv_sim2, including the core library.
#
# This also creates the variable asv_sim2_ALL_LIBRARIES
#
################################################################################

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(asv_sim2_ALL_CONFIG_INCLUDED)
  return()
endif()
set(asv_sim2_ALL_CONFIG_INCLUDED TRUE)

if(NOT asv_sim2-all_FIND_QUIETLY)
  message(STATUS "Looking for all libraries of asv_sim2 -- found version 2.0.0")
endif()


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was gz-all-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# Get access to the find_dependency utility
include(CMakeFindDependencyMacro)

# Find the core library
find_dependency(asv_sim2 2.0.0 EXACT)

# Find the component libraries
find_dependency(asv_sim2-anemometer-system 2.0.0 EXACT)
find_dependency(asv_sim2-foil-lift-drag-system 2.0.0 EXACT)
find_dependency(asv_sim2-mooring-system 2.0.0 EXACT)
find_dependency(asv_sim2-sail-lift-drag-system 2.0.0 EXACT)
find_dependency(asv_sim2-sail-position-controller-system 2.0.0 EXACT)
find_dependency(asv_sim2-wind-system 2.0.0 EXACT)

if(NOT TARGET asv_sim2::asv_sim2-all)
  include("${CMAKE_CURRENT_LIST_DIR}/asv_sim2-all-targets.cmake")

  add_library(asv_sim2::all INTERFACE IMPORTED)
  set_target_properties(asv_sim2::all PROPERTIES
    INTERFACE_LINK_LIBRARIES "asv_sim2::asv_sim2-all")

endif()

# Create the "requested" target if it does not already exist
if(NOT TARGET asv_sim2::requested)
  add_library(asv_sim2::requested INTERFACE IMPORTED)
endif()

# Link the "all" target to the "requested" target
get_target_property(gz_requested_components asv_sim2::requested INTERFACE_LINK_LIBRARIES)
if(NOT gz_requested_components)
  set_target_properties(asv_sim2::requested PROPERTIES
    INTERFACE_LINK_LIBRARIES "asv_sim2::asv_sim2-all")
else()
  set_target_properties(asv_sim2::requested PROPERTIES
    INTERFACE_LINK_LIBRARIES "${gz_requested_components};asv_sim2::asv_sim2-all")
endif()

set(asv_sim2_ALL_LIBRARIES asv_sim2::asv_sim2-all)
