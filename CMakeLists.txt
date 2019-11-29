
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS " -g -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}" )

if ( NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY )
    message( "TYSOCDART> Sending generated libs to OWN build directory" )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/libs" )
else()
    message( "TYSOCDART> Sending generated libs to EXTERNAL build directory" )
endif()

message( "TYSOCDART> LIBRARY_OUT_DIR: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" )

set( TYSOCDART_IS_MASTER_PROJECT OFF )
if( CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR )
    message( "TYSOCDART> Configuring as MASTER project" )
    set( TYSOCDART_IS_MASTER_PROJECT ON )
else()
    message( "TYSOCDART> Configuring as CHILD project" )
endif()

# Add core targets only if master project. If not, parent project must configure it
if ( TYSOCDART_IS_MASTER_PROJECT )
    # Configure dependencies for tysoc
    add_subdirectory( ext )
    # Configure core functionality
    add_subdirectory( core )
endif()

# Configure TYSOCDART project ************************************************

# Set the lbirary paths as props to allow usage from other projects
set_property( GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_DART "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtysocPhysicsDart.so" )

# get properties defined in core library
get_property( VAR_TYSOC_PATH_RESOURCES              GLOBAL PROPERTY PROP_TYSOC_PATH_RESOURCES )
get_property( VAR_TYSOC_PATH_WORKING_DIR            GLOBAL PROPERTY PROP_TYSOC_PATH_WORKING_DIR )
get_property( VAR_TYSOC_PATH_MJCF_TEMPLATES         GLOBAL PROPERTY PROP_TYSOC_PATH_MJCF_TEMPLATES )
get_property( VAR_TYSOC_PATH_URDF_TEMPLATES         GLOBAL PROPERTY PROP_TYSOC_PATH_URDF_TEMPLATES )
get_property( VAR_TYSOC_PATH_RLSIM_TEMPLATES        GLOBAL PROPERTY PROP_TYSOC_PATH_RLSIM_TEMPLATES )
get_property( VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ      GLOBAL PROPERTY PROP_TYSOC_BACKEND_GRAPHICS_GLVIZ )
get_property( VAR_TYSOC_BACKEND_PHYSICS_DART        GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_DART )

# repeat definitions from core library, as they are not propagated up
add_definitions( -DTYSOC_PATH_RESOURCES="${VAR_TYSOC_PATH_RESOURCES}" )
add_definitions( -DTYSOC_PATH_WORKING_DIR="${VAR_TYSOC_PATH_WORKING_DIR}" )
add_definitions( -DTYSOC_PATH_MJCF_TEMPLATES="${VAR_TYSOC_PATH_MJCF_TEMPLATES}" )
add_definitions( -DTYSOC_PATH_URDF_TEMPLATES="${VAR_TYSOC_PATH_URDF_TEMPLATES}" )
add_definitions( -DTYSOC_PATH_RLSIM_TEMPLATES="${VAR_TYSOC_PATH_RLSIM_TEMPLATES}" )
add_definitions( -DTYSOC_BACKEND_GRAPHICS_GLVIZ="${VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ}" )
add_definitions( -DTYSOC_BACKEND_PHYSICS_DART="${VAR_TYSOC_BACKEND_PHYSICS_DART}" )

# test all properties
message( "PROP_TYSOC_PATH_RESOURCES:            ${VAR_TYSOC_PATH_RESOURCES}" )
message( "PROP_TYSOC_PATH_WORKING_DIR:          ${VAR_TYSOC_PATH_WORKING_DIR}" )
message( "PROP_TYSOC_PATH_MJCF_TEMPLATES:       ${VAR_TYSOC_PATH_MJCF_TEMPLATES}" )
message( "PROP_TYSOC_PATH_URDF_TEMPLATES:       ${VAR_TYSOC_PATH_URDF_TEMPLATES}" )
message( "PROP_TYSOC_PATH_RLSIM_TEMPLATES:      ${VAR_TYSOC_PATH_RLSIM_TEMPLATES}" )
message( "PROP_TYSOC_BACKEND_GRAPHICS_GLVIZ:    ${VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ}" )
message( "PROP_TYSOC_BACKEND_PHYSICS_DART:      ${VAR_TYSOC_BACKEND_PHYSICS_DART}" )

# grab all the sources for compilation
file( GLOB_RECURSE TYSOCDART_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )
message( "TYSOCDART_SOURCES: ${TYSOCDART_SOURCES}" )

# add this project headers to includepath
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include/" )

# include_dirs for tysoc-core dependency: either case (master or non-master project) ... 
# we should find it at root level
include_directories( "${CMAKE_SOURCE_DIR}/core/include" )
include_directories( "${CMAKE_SOURCE_DIR}/core/ext/json" )
include_directories( "${CMAKE_SOURCE_DIR}/core/ext/tinyxml2" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/spdlog/include" )

# include dart src, which can be found in either this repo's ext ...
# folder, or in the master's repo ext folder
include_directories( "${CMAKE_SOURCE_DIR}/ext/dart" )
include_directories( "${CMAKE_BINARY_DIR}/ext/dart" )
# required for bullet-collision detector (needs bullet installed) @todo: replace properly
include_directories( "/usr/include/bullet" )

find_package( assimp REQUIRED )

# Configure the resulting library
add_library( tysocPhysicsDart SHARED
             ${TYSOCDART_SOURCES} )
target_link_libraries( tysocPhysicsDart
                       tysocCore
                       assimp
                       dart
                       dart-collision-bullet )

# ******************************************************************************

# Only generate samples if not using as dependency
if ( TYSOCDART_IS_MASTER_PROJECT )
    # Configure examples
    add_subdirectory( examples )
    # Configure tests
    add_subdirectory( tests )
endif()