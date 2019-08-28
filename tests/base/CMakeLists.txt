
message( "Configuring tysoc-dart::base_test_interface" )

include_directories( "${CMAKE_SOURCE_DIR}/ext/dart" )
include_directories( "${CMAKE_BINARY_DIR}/ext/dart" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/cat1/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui/examples" )
include_directories( "${CMAKE_SOURCE_DIR}/core/include" )

add_library( test_interface SHARED
             test_interface.cpp )
target_link_libraries( test_interface
                       tysocCore
                       cat1
                       imgui
                       dart )