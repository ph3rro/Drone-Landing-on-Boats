# - Config file for the gz-waves1 package.
#
# For finding and loading gz-waves1 from your project, type:
#
# find_package(gz-waves1)
#
# This creates the following targets:
#
#   Core library target                     - gz-waves1::gz-waves1
#   Alternative target name                 - gz-waves1::core
#   Core library + all requested components - gz-waves1::requested
#
# Use target_link_libraries() to link your library or executable to one of the
# above targets.
#
# We also provide the following variables for backwards compatibility, but use
# of these is discouraged:
#
#   gz-waves1_CORE_LIBRARY - Core library (actually contains gz-waves1::core)
#   gz-waves1_LIBRARIES    - All libraries to link against (actually contains gz-waves1::requested).
#   gz-waves1_INCLUDE_DIRS - Include directories for gz-waves1 and its dependencies.
#
# We will also set gz-waves1_FOUND to indicate that the package was found.
#
################################################################################

# We explicitly set the desired cmake version to ensure that the policy settings
# of users or of toolchains do not result in the wrong behavior for our modules.
# Note that the call to find_package(~) will PUSH a new policy stack before
# taking on these version settings, and then that stack will POP after the
# find_package(~) has exited, so this will not affect the cmake policy settings
# of a caller.
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(NOT gz-waves1_FIND_QUIETLY)
  message(STATUS "Looking for gz-waves1 -- found version 1.0.0")
endif()

if(gz-waves1_CONFIG_INCLUDED)

  # Check that each of the components requested in this call to find_package(~)
  # have already been found.
  set(gz-waves1_all_requested_components_found true)
  foreach(component ${gz-waves1_FIND_COMPONENTS})
    if(NOT gz-waves1-${component}_FOUND)
      set(gz-waves1_all_requested_components_found false)
      break()
    endif()
  endforeach()

  # If this gz-waves1-config.cmake file has been called before,
  # and all the requested components are already found, then exit early.
  if(gz-waves1_all_requested_components_found)
    return()
  endif()
endif()
set(gz-waves1_CONFIG_INCLUDED TRUE)

# Get access to the find_dependency utility
include(CMakeFindDependencyMacro)
# Find gz-cmake, because we need its modules in order to find the rest of
# our dependencies.
find_dependency(gz-cmake3)

# Set the REQUIRED flag for the find_package(~) calls on this project's
# dependencies.
if(gz-waves1_FIND_REQUIRED)
  set(gz_package_required REQUIRED)
else()
  set(gz_package_required "")
endif()

# Set the QUIET flag for the find_package(~) calls on this project's
# dependencies.
if(gz-waves1_FIND_QUIETLY)
  set(gz_package_quiet QUIET)
else()
  set(gz_package_quiet "")
endif()

# --------------------------------
# Find each required dependency of this project (if nothing is below, then the
# project has no external dependencies). We use find_package(~) instead of
# find_dependency(~) here so that we can support COMPONENT arguments.
#
# TODO: When we migrate to cmake-3.9+, change these to find_dependency(~),
#       because at that point the find_dependency(~) function will support
#       the COMPONENT argument.
if(NOT gz-waves1_FIND_QUIETLY)
  message(STATUS "Searching for dependencies of gz-waves1")
endif()
find_package(gz-math7 ${gz_package_quiet} ${gz_package_required} COMPONENTS eigen3)
find_package(gz-plugin2 ${gz_package_quiet} ${gz_package_required} COMPONENTS loader;register)
find_package(gz-common5 ${gz_package_quiet} ${gz_package_required} COMPONENTS graphics;events)
find_package(gz-msgs10 ${gz_package_quiet} ${gz_package_required})
find_package(gz-transport13 ${gz_package_quiet} ${gz_package_required})
find_package(gz-rendering8 ${gz_package_quiet} ${gz_package_required})
find_package(gz-sim8 ${gz_package_quiet} ${gz_package_required})
find_package(sdformat14 ${gz_package_quiet} ${gz_package_required})
# --------------------------------


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was gz-config.cmake.in                            ########

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

if(NOT TARGET gz-waves1::gz-waves1)
  include("${CMAKE_CURRENT_LIST_DIR}/gz-waves1-targets.cmake")

  # Create a simplified imported target name for the core library.
  # You can link to this target instead of the core library.
  add_library(gz-waves1::core INTERFACE IMPORTED)
  set_target_properties(gz-waves1::core PROPERTIES
    INTERFACE_LINK_LIBRARIES gz-waves1::gz-waves1)
  # Note: In a future version of cmake, we can replace this with an ALIAS target

  # In case someone tries to link against the plain library name, we want to
  # intercept that and have them link against a target instead. This is
  # effectively the same as gz-waves1::core, but it does not have the
  # benefit of unambiguously being a target name.
  add_library(gz-waves1 INTERFACE IMPORTED)
  set_target_properties(gz-waves1 PROPERTIES
    INTERFACE_LINK_LIBRARIES gz-waves1::gz-waves1)

endif()

# Create the "all" target if it does not already exist
if(NOT TARGET gz-waves1::requested)
  add_library(gz-waves1::requested INTERFACE IMPORTED)
endif()

# Link the core library to the "all" target. We set the property explicitly
# because target_link_libraries cannot be called on an imported target.
get_target_property(gz_all_components gz-waves1::requested INTERFACE_LINK_LIBRARIES)
if(NOT gz_all_components)
  # If gz-waves1::requested has not been given any libraries yet, then cmake will
  # set gz_all_components to gz_all_components-NOTFOUND, which is something we
  # should NOT pass into the INTERFACE_LINK_LIBRARIES property.
  set_target_properties(gz-waves1::requested PROPERTIES
    INTERFACE_LINK_LIBRARIES "gz-waves1::gz-waves1")
else()
  set_target_properties(gz-waves1::requested PROPERTIES
    INTERFACE_LINK_LIBRARIES "${gz_all_components};gz-waves1::gz-waves1")
endif()

# Package variables. Note that gz-waves1_LIBRARIES and gz-waves1_CORE_LIBRARY
# contain imported targets, so gz-waves1_INCLUDE_DIRS is never needed.
set(gz-waves1_CORE_LIBRARY gz-waves1::core)
set(gz-waves1_LIBRARIES gz-waves1::requested)

if (NOT gz-waves1_INCLUDE_DIRS_OVERRIDE)
  set_and_check(gz-waves1_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include/gz/waves1")
else()
  set_and_check(gz-waves1_INCLUDE_DIRS ${gz-waves1_INCLUDE_DIRS_OVERRIDE})
endif()

# Backwards compatibility variables
set(GZ-WAVES_LIBRARIES ${gz-waves1_LIBRARIES})
set(GZ-WAVES_INCLUDE_DIRS ${gz-waves1_INCLUDE_DIRS})

# This macro is used by gz-cmake to automatically configure the pkgconfig
# files for Gazebo projects.
gz_pkg_config_entry(gz-waves1 "gz-waves1")

# Find each of the components requested by find_package(~)
foreach(component ${gz-waves1_FIND_COMPONENTS})

  if(NOT gz-waves1_FIND_QUIETLY)
    message(STATUS "Searching for <gz-waves1> component [${component}]")
  endif()

  if(gz-waves1_FIND_REQUIRED_${component})

    # Find the component package using find_dependency(~). If the user specified
    # REQUIRED or QUIET, those will automatically get forwarded to
    # find_dependency(~)
    find_dependency(gz-waves1-${component} 1.0.0 EXACT)

  else()

    # If this is an optional component, use find_package(~) instead of
    # find_dependency(~) so we can ensure that the use of REQUIRED does not get
    # forwarded to it.
    find_package(gz-waves1-${component} 1.0.0 EXACT ${gz_package_quiet})

  endif()

endforeach()

# Specify the doxygen tag file
set(GZ-WAVES_DOXYGEN_TAGFILE "${PACKAGE_PREFIX_DIR}/share/gz/gz-waves1/gz-waves1.tag.xml")

# Specify the API url. This is where the doxygen tag file will resolve URLS to.
set(GZ-WAVES_API_URL "https://gazebosim.org/api/waves/1")

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${gz-waves1_DIR}/${_extra}")
endforeach()
