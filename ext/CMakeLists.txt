
# Configure dependencies
message( "TYSOCDART> Configuring external dependencies" )

# build spdlog as a shared library
set( SPDLOG_BUILD_SHARED ON CACHE BOOL "Build shared library" )

# build tiny_math without examples
set( TINYMATH_BUILD_DOCS OFF CACHE BOOL "Don't build tinymath's documentation" )
set( TINYMATH_BUILD_EXAMPLES OFF CACHE BOOL "Don't build tinymath's examples" )
set( TINYMATH_BUILD_PYTHON_BINDINGS ON CACHE BOOL "Build tinymath's python-bindings" )

# build tiny_renderer without examples
set( CAT1_BUILD_EXAMPLES OFF CACHE BOOL "Do not build tiny_renderer examples" )

add_subdirectory( imgui )
add_subdirectory( spdlog )
add_subdirectory( pybind11 )
add_subdirectory( tiny_math )
add_subdirectory( tiny_renderer )

# remove DART examples from build
set( DART_BUILD_GUI_OSG OFF CACHE BOOL "Build osgDart library" )
set( DART_BUILD_EXAMPLES OFF  CACHE BOOL "Build dart-examples" )
set( DART_BUILD_TUTORIALS OFF CACHE BOOL "Build dart-tutorials" )
set( DART_BUILD_TESTS OFF CACHE BOOL "Build dart-tests" )
set( DART_VERBOSE ON CACHE BOOL "Build with full logs")

add_subdirectory( dart )