
# Configure dependencies
message( "TYSOCDART> Configuring external dependencies" )

add_subdirectory( cat1 )
add_subdirectory( imgui )
add_subdirectory( pybind11 )

# remove DART examples from build
set( DART_BUILD_GUI_OSG OFF CACHE BOOL "Build osgDart library" )
set( DART_BUILD_EXAMPLES OFF  CACHE BOOL "Build dart-examples" )
set( DART_BUILD_TUTORIALS OFF CACHE BOOL "Build dart-tutorials" )
set( DART_BUILD_TESTS OFF CACHE BOOL "Build dart-tests" )
set( DART_VERBOSE ON CACHE BOOL "Build with full logs")

add_subdirectory( dart )